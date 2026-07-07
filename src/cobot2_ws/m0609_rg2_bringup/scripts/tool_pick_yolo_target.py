import math
import struct
import time
import cv2

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PlanningOptions, RobotState,
    PositionConstraint, OrientationConstraint, JointConstraint,
    CollisionObject, AttachedCollisionObject,
)
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from onrobot_rg_msgs.srv import SetCommand

# 최종 하강(grasp)은 관절공간 IK 이동 대신 MoveIt Cartesian Path
# (compute_cartesian_path + ExecuteTrajectory)로 처리 — pregrasp/grasp를 각각
# solve_ik로 따로 풀면 두 IK가 서로 다른 특이점 회피 틸트를 고를 수 있어 그 사이를
# 관절공간으로 보간한 경로가 대각선처럼 보일 수 있음. Cartesian Path는 직교좌표상
# 직선 경로를 직접 보장함.
# ※ 두산 네이티브 movel/get_current_posx(DSR_ROBOT2.py, dsr_controller2 서비스)는
# 이 프로젝트의 bringup(_camera).launch.py가 dsr_controller2 컨트롤러를 스폰하지
# 않고 일반 joint_trajectory_controller(dsr_moveit_controller)만 띄우기 때문에
# 서비스 자체가 존재하지 않아 사용 불가 (MoveIt Execute를 살리려고 의도적으로
# 그렇게 설정된 것으로 보임 — 되돌리면 MoveIt 쪽이 깨질 수 있어 시도하지 않음).

MODEL_PATH = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/voice_processing/resource/toolbest.pt'
CLASS_NAMES = {0: 'screw2', 1: 'tool-hammer'}
# 단일 고정 타겟 대신, 화면에 보이는 것 중(KEYPOINT_AXIS_ROLE에 등록된 클래스면
# 전부 후보) 컨피던스가 가장 높은 것을 골라서 잡는다.
CONFIDENCE_THRESHOLD = 0.7
DETECT_TIMEOUT_SEC = 15.0
MIN_DEPTH_M = 0.1
MAX_DEPTH_M = 3.0

PLANNING_FRAME = 'base_link'
GROUP_NAME = 'manipulator'
EEF_LINK = 'rg2_tcp'
# ※ manipulator 그룹 tip_link = rg2_tcp (m0609_rg2.srdf). TCP 오프셋(tool0 기준
# 회전 없이 Z 231.066mm, 2026-07-07 두산 티칭펜던트 실측)은 onrobot_rg2_model_macro.xacro의
# fixed joint로 URDF에 반영되어 있어, MoveIt이 그리퍼 손끝을 직접 목표로 잡는다.
SPEED_SCALE = 0.1              # 낮을수록 천천히 (가상모드라도 우선 저속 유지)
PREGRASP_Z_OFFSET = 0.15       # 물체 위 접근 높이 (m)
GRASP_Z_CLEARANCE = 0.02       # 계산된 표면점보다 살짝 위에서 잡기
CARTESIAN_MAX_STEP = 0.01      # pregrasp -> grasp 하강 Cartesian Path 보간 간격 (m)

# 매 실행 시작 시 이동할 홈 자세(관절, degree). robot_control.py의 init_robot()에
# 있는 범용 JReady와 동일값 (jenga_inspector.py의 JReady는 젠가 스캔 전용 포즈라 다름).
HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# move_group만 단독 실행(moveit_camera.launch.py)할 때는 controller_manager가
# 없어 실제 Execute/그리퍼 서비스가 불가하므로 기본값 True/False.
# bringup까지 붙여 실제로 움직이고 싶을 때만 False/True로 바꿀 것.
PLAN_ONLY = False
USE_GRIPPER = True

# top-down(그리퍼가 아래를 보는) 자세 — YAW는 keypoint 축으로 매 탐지마다 계산해서 대체함
GRASP_ROLL, GRASP_PITCH = math.pi, 0.0

# 라벨링 keypoint 순서(머리 2점 + 꼬리 1점, pad_keypoints.py 기준):
# tool-hammer: [head2_1, head2_2, tail2] -> 축 = mean(head2_1, head2_2) <-> tail2
# screw2:      [head1, tail1, (미사용 패딩)] -> 축 = head1 <-> tail1
KEYPOINT_AXIS_ROLE = {
    'tool-hammer': {'head_idx': [0, 1], 'tail_idx': 2},
    'screw2': {'head_idx': [0], 'tail_idx': 1},
}


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rot_matrix_from_rpy(roll, pitch, yaw):
    """quat_from_rpy와 동일한 축 규약(R = Rz(yaw)*Ry(pitch)*Rx(roll))의 3x3 행렬."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def compute_attach_pose(flatten_roll, flatten_pitch, world_yaw_offset, native_offset,
                         axis_angle, yaw, detected_xyz):
    """메쉬를 눕히는 고정 회전(flatten_roll/pitch)과 물체 실제 방향(axis_angle)으로 world
    자세를 구성하고, native_offset(메쉬 원점 기준 "길이 중심" 등 정렬 기준점, 원점이
    이미 중심이면 (0,0,0))이 검출 좌표(detected_xyz)/그리퍼 원점에 오도록 위치까지
    함께 계산한다. 그리퍼 부착용 자세는 world 자세를 tcp의 실제 world 회전(GRASP_ROLL/
    PITCH, grasp용 yaw)으로 역회전시켜 tcp 로컬 기준으로 구한다.
    각도 하나만으로 긴 축만 맞추면 그 축에서 벗어난 부분(예: 해머 머리)이 반대로
    뒤집힐 수 있어서, 회전행렬 전체로 위치/자세를 함께 유도한다."""
    world_yaw = axis_angle + world_yaw_offset
    r_world = rot_matrix_from_rpy(flatten_roll, flatten_pitch, world_yaw)
    r_tcp = rot_matrix_from_rpy(GRASP_ROLL, GRASP_PITCH, yaw)
    r_local = r_tcp.T @ r_world

    offset = np.array(native_offset)
    world_position = tuple(np.array(detected_xyz) - r_world @ offset)
    gripper_position = tuple(-(r_local @ offset))

    world_quat = quat_from_matrix(r_world)
    gripper_quat = quat_from_matrix(r_local)
    return world_position, world_quat, gripper_position, gripper_quat


def quat_from_matrix(r):
    """3x3 회전행렬 -> 쿼터니언(x,y,z,w). 단일 회전각(roll/pitch/yaw 중 하나)만으로는
    표현 안 되는 임의 회전(예: 두 좌표계 사이의 상대 회전)을 넘길 때 사용."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (r[2, 1] - r[1, 2]) * s
        qy = (r[0, 2] - r[2, 0]) * s
        qz = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def _add_vertex(mesh, vertices_map, current_triangle, x, y, z, scale):
    pt_key = (round(x * scale, 6), round(y * scale, 6), round(z * scale, 6))
    if pt_key not in vertices_map:
        p = Point()
        p.x, p.y, p.z = pt_key
        vertices_map[pt_key] = len(mesh.vertices)
        mesh.vertices.append(p)
    current_triangle.append(vertices_map[pt_key])
    if len(current_triangle) == 3:
        triangle = MeshTriangle()
        triangle.vertex_indices = current_triangle[:]
        mesh.triangles.append(triangle)
        current_triangle.clear()


def load_stl_mesh(filepath, scale=1.0):
    """ASCII/바이너리 STL을 모두 지원. scale은 파일 좌표 단위를 미터로
    맞추기 위한 배율(예: mm 단위로 내보낸 스캔이면 0.001)."""
    mesh = Mesh()
    vertices_map = {}
    current_triangle = []

    with open(filepath, 'rb') as f:
        data = f.read()

    is_binary = False
    if len(data) >= 84:
        tri_count = struct.unpack('<I', data[80:84])[0]
        if 84 + tri_count * 50 == len(data):
            is_binary = True

    if is_binary:
        offset = 84
        for _ in range(tri_count):
            record = data[offset:offset + 50]
            vals = struct.unpack('<12fH', record)
            for k in (3, 6, 9):  # normal(0:3) 건너뛰고 v1, v2, v3만 사용
                _add_vertex(mesh, vertices_map, current_triangle,
                            vals[k], vals[k + 1], vals[k + 2], scale)
            offset += 50
    else:
        for line in data.decode('utf-8', errors='ignore').splitlines():
            line = line.strip()
            if not line.startswith('vertex'):
                continue
            _, xs, ys, zs = line.split()
            _add_vertex(mesh, vertices_map, current_triangle,
                        float(xs), float(ys), float(zs), scale)

    return mesh


def scale_trajectory_speed(trajectory, scale):
    """RobotTrajectory의 시간축을 늘려 실행 속도를 scale배로 낮춘다(1.0=원래 속도).
    compute_cartesian_path 결과는 속도 스케일 옵션이 없어 기본 속도로 계산되므로,
    실기에서 SPEED_SCALE만큼 느리게 실행되도록 실행 직전에 재타이밍한다."""
    for point in trajectory.joint_trajectory.points:
        t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        t /= scale
        sec = int(t)
        nanosec = int(round((t - sec) * 1e9))
        if nanosec >= 1_000_000_000:  # 반올림으로 1초를 넘어가는 경우 보정
            sec += 1
            nanosec -= 1_000_000_000
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        point.velocities = [v * scale for v in point.velocities]
        point.accelerations = [a * scale * scale for a in point.accelerations]
    return trajectory


class PickYoloTarget(Node):

    def __init__(self):
        super().__init__('pick_yolo_target')
        self.bridge = CvBridge()

        # Tool weight(CamWeight, 2026-07-07 펜던트 실측 0.420kg) — 등록된 Tool weight
        # 값 자체를 돌려주는 서비스가 없어(get_workpiece_weight는 Tool 설정 이후 "추가로"
        # 든 무게만 측정) 로봇에서 조회 불가. 참고/로깅용 파라미터로만 유지.
        self.declare_parameter('tool_weight_kg', 0.420)
        self.tool_weight_kg = self.get_parameter('tool_weight_kg').get_parameter_value().double_value

        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO device: {self.device}')
        self.model = YOLO(MODEL_PATH)
        self.model.to(self.device)

        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None
        self.camera_frame_id = None
        self.current_joint6 = None
        self.current_joint_state = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # YOLO 탐지 시각화 이미지 퍼블리셔 (rqt_image_view로 확인)
        self._vis_pub = self.create_publisher(Image, '/yolo_detection/image', 1)

        # MoveIt PlanningScene 관리용 퍼블리셔
        self._collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self._attached_collision_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)

        self.tf_buffer = tf2_ros.Buffer()
        # TransformListener 전용 노드를 분리하고 백그라운드 스레드에서 스핀하도록 설정 (콜백 큐 간섭 방지)
        self.tf_node = Node('tf_listener_node')
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)

        self._move_ac = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('move_group 대기중...')
        while not self._move_ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  ... move_group 응답 없음, 재시도 중...')
            rclpy.spin_once(self, timeout_sec=0.1)

        self._gripper_cli = None
        if USE_GRIPPER:
            self._gripper_cli = self.create_client(SetCommand, '/onrobot/sendCommand')
            self.get_logger().info('/onrobot/sendCommand 대기중...')
            while not self._gripper_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('  ... /onrobot/sendCommand 응답 없음, 재시도 중...')
                rclpy.spin_once(self, timeout_sec=0.1)

        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('/compute_ik 대기중...')
        while not self._ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ... /compute_ik 응답 없음, 재시도 중...')
            rclpy.spin_once(self, timeout_sec=0.1)

        self._cartesian_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.get_logger().info('/compute_cartesian_path 대기중...')
        while not self._cartesian_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ... /compute_cartesian_path 응답 없음, 재시도 중...')
            rclpy.spin_once(self, timeout_sec=0.1)

        self._execute_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.get_logger().info('/execute_trajectory 대기중...')
        while not self._execute_ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  ... /execute_trajectory 응답 없음, 재시도 중...')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('준비 완료')

    def destroy_node(self):
        self.tf_node.destroy_node()
        super().destroy_node()

    def _color_cb(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.camera_frame_id = msg.header.frame_id

    def _depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _camera_info_cb(self, msg):
        self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'ppx': msg.k[2], 'ppy': msg.k[5]}

    def _joint_state_cb(self, msg):
        self.current_joint_state = msg
        if 'joint_6' in msg.name:
            self.current_joint6 = msg.position[msg.name.index('joint_6')]

    def _sample_depth(self, cx_px, cy_px):
        h, w = self.depth_frame.shape[:2]
        if not (0 <= cy_px < h and 0 <= cx_px < w):
            return None
        y0, y1 = max(0, cy_px - 2), min(h, cy_px + 3)
        x0, x1 = max(0, cx_px - 2), min(w, cx_px + 3)
        patch = self.depth_frame[y0:y1, x0:x1].astype(np.float32)
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        depth_m = float(np.median(patch)) / 1000.0
        if not (MIN_DEPTH_M <= depth_m <= MAX_DEPTH_M):
            return None
        return depth_m

    def _visualize(self, frame, results, best):
        """탐지 결과를 OpenCV 윈도우에 시각화."""
        vis = frame.copy()
        # 모든 박스 그리기
        for box_xyxy, score, label in zip(
            results.boxes.xyxy.tolist(), results.boxes.conf.tolist(), results.boxes.cls.tolist()
        ):
            x1, y1, x2, y2 = map(int, box_xyxy)
            class_name = CLASS_NAMES.get(int(label), f'class_{int(label)}')
            is_target = (best is not None and class_name in KEYPOINT_AXIS_ROLE and score >= CONFIDENCE_THRESHOLD)
            color = (0, 255, 0) if is_target else (128, 128, 128)  # 타겟=녹색, 기타=회색
            thickness = 3 if is_target else 1
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(vis, f'{class_name} {score:.2f}', (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)
        # 타겟 키포인트 그리기
        if best is not None:
            _, (bx1, by1, bx2, by2), bidx, best_class = best
            cx, cy = int((bx1 + bx2) / 2), int((by1 + by2) / 2)
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)  # 중심점 빨간 원
            role = KEYPOINT_AXIS_ROLE[best_class]
            kpts_xy = results.keypoints.xy[bidx].tolist()
            for kidx, kpt in enumerate(kpts_xy):
                kx, ky = int(kpt[0]), int(kpt[1])
                if kx == 0 and ky == 0:
                    continue
                kcolor = (255, 0, 0) if kidx in role['head_idx'] else (0, 165, 255)
                cv2.circle(vis, (kx, ky), 5, kcolor, -1)
            # 축 선 그리기 (head → tail)
            head_px = np.mean([kpts_xy[j] for j in role['head_idx']], axis=0)
            tail_px = kpts_xy[role['tail_idx']]
            cv2.line(vis, (int(head_px[0]), int(head_px[1])),
                     (int(tail_px[0]), int(tail_px[1])), (0, 255, 255), 2)
        # 임계값 표시
        targets_label = '/'.join(KEYPOINT_AXIS_ROLE.keys())
        cv2.putText(vis, f'Threshold: {CONFIDENCE_THRESHOLD:.0%}  Target: {targets_label}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # ROS 토픽으로 발행 (rqt_image_view에서 /yolo_detection/image 구독)
        # cv_bridge + OpenCV5 호환 문제 우회: Image 메시지 직접 생성
        from sensor_msgs.msg import Image as ImageMsg
        vis_u8 = np.ascontiguousarray(vis, dtype=np.uint8)
        msg = ImageMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = vis_u8.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = vis_u8.tobytes()
        self._vis_pub.publish(msg)

    def detect_once(self, timeout_sec=DETECT_TIMEOUT_SEC):
        """color/depth/intrinsics가 갖춰지면, 화면에 보이는 픽업 대상(KEYPOINT_AXIS_ROLE에
        등록된 클래스) 중 최고 confidence 탐지 1건을 클래스 구분 없이 반환."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.color_frame is None or self.depth_frame is None or self.intrinsics is None:
                continue

            results = self.model(self.color_frame, device=self.device, verbose=False)[0]
            best = None
            for idx, (box_xyxy, score, label) in enumerate(zip(
                results.boxes.xyxy.tolist(), results.boxes.conf.tolist(), results.boxes.cls.tolist(),
            )):
                if score < CONFIDENCE_THRESHOLD:
                    continue
                class_name = CLASS_NAMES.get(int(label), f'class_{int(label)}')
                if class_name not in KEYPOINT_AXIS_ROLE:
                    continue
                if best is None or score > best[0]:
                    best = (score, box_xyxy, idx, class_name)

            # 시각화 (탐지 여부와 무관하게 매 프레임 갱신)
            self._visualize(self.color_frame, results, best)

            if best is None:
                continue

            score, (x1, y1, x2, y2), idx, class_name = best
            cx_px, cy_px = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_m = self._sample_depth(cx_px, cy_px)
            if depth_m is None:
                continue

            role = KEYPOINT_AXIS_ROLE[class_name]
            kpts_xy = results.keypoints.xy[idx].tolist()
            head_px = np.mean([kpts_xy[j] for j in role['head_idx']], axis=0)
            tail_px = np.array(kpts_xy[role['tail_idx']])

            self.get_logger().info(f'{class_name} 탐지 (conf={score:.2f}, depth={depth_m:.3f}m)')
            return cx_px, cy_px, depth_m, head_px, tail_px, class_name

        return None

    def pixel_to_base_xyz(self, cx_px, cy_px, depth_m):
        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']
        x = (cx_px - ppx) * depth_m / fx
        y = (cy_px - ppy) * depth_m / fy

        pose_camera = Pose()
        pose_camera.position.x = x
        pose_camera.position.y = y
        pose_camera.position.z = depth_m
        pose_camera.orientation.w = 1.0

        # DDS transient local (/tf_static) 디스커버리 및 버퍼링 시간 확보를 위한 재시도 루프
        import tf2_ros
        max_retries = 10
        for i in range(max_retries):
            try:
                transform = self.tf_buffer.lookup_transform(
                    PLANNING_FRAME, self.camera_frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                if i == max_retries - 1:
                    self.get_logger().error(f"TF 조회 최종 실패: {e}")
                    raise e
                self.get_logger().warn(f"TF 조회 대기 중 ({i+1}/{max_retries}): {e}")
                rclpy.spin_once(self, timeout_sec=1.0)
        
        pose_base = do_transform_pose(pose_camera, transform)
        return pose_base.position.x, pose_base.position.y, pose_base.position.z

    def build_pose(self, x, y, z, yaw):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        qx, qy, qz, qw = quat_from_rpy(GRASP_ROLL, GRASP_PITCH, yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose

    def axis_yaw(self, head_px, tail_px):
        """head<->tail keypoint 축을 base_link 수평면 각도로 변환.
        (axis_angle, yaw) 튜플로 반환:
          - axis_angle: 물체의 실제 방향(머리<-꼬리, 180도 뒤집기 없음). 부착 메쉬
            방향처럼 머리/꼬리 비대칭 형상을 정확히 정렬해야 하는 곳에 사용.
          - yaw: 그리퍼 목표 자세용. 평행 2핑거 그리퍼는 yaw와 yaw+180도*k가 파지
            결과는 같으므로, joint_6를 불필요하게 크게 돌리지 않도록 axis_angle에
            180도의 정수배를 더해 현재 joint_6와 가장 가까운 값을 선택한 것."""
        head_depth = self._sample_depth(int(round(head_px[0])), int(round(head_px[1])))
        tail_depth = self._sample_depth(int(round(tail_px[0])), int(round(tail_px[1])))
        if head_depth is None or tail_depth is None:
            self.get_logger().warn('축 keypoint depth 없음 — yaw=0(기본 top-down)으로 대체')
            return 0.0, 0.0

        hx, hy, _ = self.pixel_to_base_xyz(int(round(head_px[0])), int(round(head_px[1])), head_depth)
        tx, ty, _ = self.pixel_to_base_xyz(int(round(tail_px[0])), int(round(tail_px[1])), tail_depth)
        axis_angle = math.atan2(hy - ty, hx - tx)
        axis_angle = math.atan2(math.sin(axis_angle), math.cos(axis_angle))  # -180~180도로 정규화
        yaw = axis_angle

        if self.current_joint6 is None:
            # 현재 값을 모르면 기본 -90~90도로 접어서 반환
            if yaw > math.pi / 2:
                yaw -= math.pi
            elif yaw <= -math.pi / 2:
                yaw += math.pi
            return axis_angle, yaw

        candidates = [yaw + k * math.pi for k in range(-3, 4)]
        yaw = min(candidates, key=lambda c: abs(c - self.current_joint6))
        return axis_angle, yaw

    def move_to_pose(self, pose: Pose, speed_scale=SPEED_SCALE):
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = EEF_LINK
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [pose]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = EEF_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 0.3
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]

        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = speed_scale
        req.max_acceleration_scaling_factor = speed_scale

        opts = PlanningOptions()
        opts.plan_only = PLAN_ONLY

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        self.get_logger().info(
            f'이동: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
        send_future = self._move_ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 move_group에서 거부됨')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('플래닝 성공 (RViz에서 궤적 미리보기 확인)' if PLAN_ONLY else '이동 완료')
            return True
        self.get_logger().error(f'실패: error_code={result.error_code.val}')
        return False

    def gripper_command(self, command: str):
        if not USE_GRIPPER:
            self.get_logger().info(f'(USE_GRIPPER=False, 그리퍼 명령 생략: {command!r})')
            return True
        req = SetCommand.Request()
        req.command = command
        future = self._gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None or not res.success:
            self.get_logger().error(f'그리퍼 명령 실패: {command!r}')
        return res is not None and res.success

    def solve_ik(self, x, y, z, yaw, seed_joints=None) -> list[float] | None:
        """주어진 x, y, z, yaw에 대해 IK를 해결. 특이점 및 joint_5 리밋 회피를 위해 미세 틸트 순회."""
        # 틸트 오프셋 후보 (Roll, Pitch 오프셋 라디안 단위, 약 ±5도 및 ±10도)
        tilts = [
            (0.0, 0.0),          # 1. 완전 수직 top-down
            (0.0, 0.087),        # 2. Pitch +5 deg
            (0.0, -0.087),       # 3. Pitch -5 deg
            (0.087, 0.0),        # 4. Roll +5 deg
            (-0.087, 0.0),       # 5. Roll -5 deg
            (0.0, 0.174),        # 6. Pitch +10 deg
            (0.0, -0.174),       # 7. Pitch -10 deg
            (0.174, 0.0),        # 8. Roll +10 deg
            (-0.174, 0.0),       # 9. Roll -10 deg
        ]

        for d_roll, d_pitch in tilts:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            
            # 틸트 각도를 반영한 RPY 계산
            roll = GRASP_ROLL + d_roll
            pitch = GRASP_PITCH + d_pitch
            qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            req = GetPositionIK.Request()
            req.ik_request.group_name = GROUP_NAME
            req.ik_request.ik_link_name = EEF_LINK
            req.ik_request.avoid_collisions = False
            
            # 시드 조인트가 주어지면 이를 사용하고, 없으면 현재 조인트 상태를 사용 (wrist flip 방지)
            if seed_joints is not None:
                req.ik_request.robot_state.joint_state.name = [f"joint_{i}" for i in range(1, 7)]
                req.ik_request.robot_state.joint_state.position = seed_joints
            elif self.current_joint_state is not None:
                req.ik_request.robot_state.joint_state = self.current_joint_state
            
            # IK 솔버 탐색 시간
            req.ik_request.timeout = Duration(seconds=1.0).to_msg()
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = PLANNING_FRAME
            pose_stamped.pose = pose
            req.ik_request.pose_stamped = pose_stamped
            
            future = self._ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            
            if res is not None and res.error_code.val == 1:
                if d_roll != 0.0 or d_pitch != 0.0:
                    self.get_logger().info(
                        f"미세 틸트 각도 적용하여 IK 해 성공: Pitch_offset={math.degrees(d_pitch):.1f}deg, Roll_offset={math.degrees(d_roll):.1f}deg"
                    )
                
                joint_state = res.solution.joint_state
                joint_names = [f"joint_{i}" for i in range(1, 7)]
                positions = []
                for name in joint_names:
                    if name in joint_state.name:
                        idx = joint_state.name.index(name)
                        positions.append(joint_state.position[idx])
                    else:
                        self.get_logger().error(f"IK 결과에서 관절 {name}을 찾을 수 없습니다.")
                        return None
                return positions
                
        self.get_logger().error(f"모든 틸트 후보에 대해 IK 계산 실패 (x={x:.3f}, y={y:.3f}, z={z:.3f})")
        return None

    def move_to_joints(self, joint_positions: list[float], speed_scale=SPEED_SCALE):
        goal_constraints = Constraints()
        joint_names = [f"joint_{i}" for i in range(1, 7)]
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = speed_scale
        req.max_acceleration_scaling_factor = speed_scale

        opts = PlanningOptions()
        opts.plan_only = PLAN_ONLY

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        self.get_logger().info(f'관절 공간 이동: {[f"{math.degrees(p):.1f}" for p in joint_positions]} deg')
        send_future = self._move_ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 move_group에서 거부됨')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('관절 공간 이동 완료')
            return True
        self.get_logger().error(f'실패: error_code={result.error_code.val}')
        return False

    def move_cartesian(self, target_pose: Pose, speed_scale=SPEED_SCALE):
        """현재 자세에서 target_pose까지 직교좌표 직선 경로로 이동(compute_cartesian_path
        로 경로 계산 후 execute_trajectory로 실행). pregrasp -> grasp 최종 하강처럼
        관절공간 보간이 아니라 진짜 직선 경로가 필요할 때 사용.
        GetCartesianPath 서비스 자체엔 속도 스케일 옵션이 없어서, 계산된 궤적을
        실행 직전에 scale_trajectory_speed로 느리게 재타이밍한다(실물 로봇 안전)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.group_name = GROUP_NAME
        req.link_name = EEF_LINK
        req.waypoints = [target_pose]
        req.max_step = CARTESIAN_MAX_STEP
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None or res.fraction < 0.99:
            fraction = res.fraction if res is not None else 0.0
            self.get_logger().error(f'Cartesian Path 계산 실패 (fraction={fraction:.2f})')
            return False

        if PLAN_ONLY:
            self.get_logger().info('Cartesian Path 계산 완료 (PLAN_ONLY=True라 실행은 생략)')
            return True

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = scale_trajectory_speed(res.solution, speed_scale)
        send_future = self._execute_ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Cartesian 경로 실행 목표가 거부됨')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Cartesian 하강 이동 완료')
            return True
        self.get_logger().error(f'Cartesian 경로 실행 실패: error_code={result.error_code.val}')
        return False

    def spawn_attached_object(self, obj_id, mesh_msg, position, orientation_quat, link_name='base_link'):
        """position/orientation_quat(x,y,z,w)은 도구별로 다른 메쉬 원점/긴 축 방향을
        반영해 호출부(run())에서 미리 계산해 넘긴다."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        aco = AttachedCollisionObject()
        aco.link_name = link_name

        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = obj_id

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position

        qx, qy, qz, qw = orientation_quat
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        co.meshes = [mesh_msg]
        co.mesh_poses = [pose]
        co.operation = CollisionObject.ADD
        
        aco.object = co
        # 터치 허용 링크 설정: 그리퍼 핑거들이 메쉬와 충돌해도 MoveIt이 에러(-27)를 내지 않도록 설정
        aco.touch_links = [
            'rg2_left_inner_finger', 'rg2_right_inner_finger',
            'rg2_left_inner_knuckle', 'rg2_right_inner_knuckle',
            'rg2_left_outer_knuckle', 'rg2_right_outer_knuckle',
            'rg2_base_link',
            'table',
            'conveyor_belt'
        ]
        
        for _ in range(5):
            self._attached_collision_pub.publish(aco)
            time.sleep(0.1)
        self.get_logger().info(f"물체 '{obj_id}'가 '{link_name}'에 부착된 상태(실제 장애물 판정)로 스폰되었습니다.")

    def attach_object_to_gripper(self, obj_id, mesh_msg, position, orientation_quat):
        """position/orientation_quat(x,y,z,w)은 도구별로 다른 메쉬 원점/긴 축 방향을
        반영해 호출부(run())에서 미리 계산해 넘긴다 (rg2_tcp 기준 로컬 좌표)."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        aco = AttachedCollisionObject()
        aco.link_name = EEF_LINK

        co = CollisionObject()
        co.header.frame_id = EEF_LINK  # 그리퍼 팁(rg2_tcp) 기준 좌표계
        co.id = obj_id

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position

        qx, qy, qz, qw = orientation_quat
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        co.meshes = [mesh_msg]
        co.mesh_poses = [pose]
        co.operation = CollisionObject.ADD
        
        aco.object = co
        aco.touch_links = [
            'rg2_left_inner_finger', 'rg2_right_inner_finger',
            'rg2_left_inner_knuckle', 'rg2_right_inner_knuckle',
            'rg2_left_outer_knuckle', 'rg2_right_outer_knuckle',
            'rg2_base_link',
            'table',
            'conveyor_belt'
        ]
        
        for _ in range(5):
            self._attached_collision_pub.publish(aco)
            time.sleep(0.1)
        self.get_logger().info(f"물체 '{obj_id}' 그리퍼에 부착 완료")
    def run(self):
        self.get_logger().info('홈 자세로 이동 중...')
        self.move_to_joints([math.radians(d) for d in HOME_JOINTS_DEG])

        targets_label = '/'.join(KEYPOINT_AXIS_ROLE.keys())
        self.get_logger().info(f'{targets_label} 탐지 대기중...')
        detection = self.detect_once()
        if detection is None:
            self.get_logger().error('타겟을 찾지 못함 (타임아웃)')
            return

        cx, cy, depth_m, head_px, tail_px, detected_class = detection
        x, y, z = self.pixel_to_base_xyz(cx, cy, depth_m)
        axis_angle, yaw = self.axis_yaw(head_px, tail_px)
        self.get_logger().info(
            f'base_link 좌표: ({x:.3f}, {y:.3f}, {z:.3f}), '
            f'axis_angle={math.degrees(axis_angle):.1f}deg, yaw={math.degrees(yaw):.1f}deg')

        # 도구별 메쉬 파일/스케일/부착 자세 계산 (compute_attach_pose가 위치+자세를
        # 회전행렬로 함께 유도 — 각도 하나만으로 긴 축만 맞추면 그 축에서 벗어난 부분이
        # 반대로 뒤집힐 수 있어서 각 도구마다 직접 검증한 파라미터를 사용).
        # - hammer.stl: 원점이 손잡이 길이 방향 중앙(native_offset=0), 손잡이=Z축/헤드=X축
        #   (서 있는 T자 형태). 롤(X축 회전)로 눕혀야 헤드(X)는 그대로 두고 손잡이(Z)만
        #   수평으로 옮겨져서 손잡이/헤드 둘 다 XY 평면에 남음(피치는 X<->Z를 섞어서 헤드가
        #   수직으로 넘어감).
        # - screwdriver.stl: 실측 스캔(mm 단위라 0.001 스케일 필요), 원점이 팁(한쪽 끝),
        #   긴 축이 Y, 길이 중심은 원점에서 -0.08m. 원형 단면이라 회전축 선택에 hammer
        #   같은 제약은 없어서 요(Z축)만으로 충분.
        if detected_class == 'tool-hammer':
            stl_path = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup/meshes/hammer.stl'
            mesh_scale = 1.0
            obj_id = 'hammer'
            world_position, world_quat, gripper_position, gripper_quat = compute_attach_pose(
                flatten_roll=math.pi / 2, flatten_pitch=0.0, world_yaw_offset=math.pi / 2,
                native_offset=(0.0, 0.0, 0.0), axis_angle=axis_angle, yaw=yaw,
                detected_xyz=(x, y, z))
        elif detected_class == 'screw2':
            stl_path = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup/meshes/screwdriver.stl'
            mesh_scale = 0.001
            obj_id = 'screwdriver'
            world_position, world_quat, gripper_position, gripper_quat = compute_attach_pose(
                flatten_roll=0.0, flatten_pitch=0.0, world_yaw_offset=-math.pi / 2,
                native_offset=(0.0, -0.08, 0.0), axis_angle=axis_angle, yaw=yaw,
                detected_xyz=(x, y, z))
        else:
            self.get_logger().error(f'미지원 detected_class: {detected_class}')
            return

        mesh_msg = None
        try:
            mesh_msg = load_stl_mesh(stl_path, scale=mesh_scale)
            self.spawn_attached_object(obj_id, mesh_msg, world_position, world_quat)
            self.get_logger().info(f'{detected_class} 검출 위치에 {obj_id} STL 스폰 완료')
        except Exception as e:
            self.get_logger().error(f'STL 스폰 실패: {e}')

        # pregrasp까지는 MoveIt 관절공간 이동(특이점 회피용 틸트 후보 탐색 포함).
        pregrasp_joints = self.solve_ik(x, y, z + PREGRASP_Z_OFFSET, yaw)
        if pregrasp_joints is None:
            self.get_logger().error('pregrasp 포즈에 대한 IK 해를 찾지 못했습니다. 기동을 취소합니다.')
            return

        if not self.gripper_command('o'):
            return
        if not self.move_to_joints(pregrasp_joints):
            return

        # 최종 하강은 Cartesian Path(직선 경로)로 처리 — pregrasp/grasp를 각각
        # solve_ik로 따로 풀어 관절공간으로 보간하면 두 IK가 서로 다른 특이점 회피
        # 틸트를 고를 수 있어 대각선처럼 보일 수 있었음. 자세(yaw)는 pregrasp와
        # 동일하게 유지한 채 Z만 낮춘 직선 경로를 직접 계산/실행한다.
        grasp_pose = self.build_pose(x, y, z + GRASP_Z_CLEARANCE, yaw)
        if not self.move_cartesian(grasp_pose):
            self.get_logger().error('Cartesian 하강 실패. 기동을 취소합니다.')
            return

        if not self.gripper_command('c'):
            return
        # 그리퍼를 닫은 후, 월드(base_link)에 있던 장애물을 그리퍼 프레임으로 리어태치
        self.attach_object_to_gripper(obj_id, mesh_msg, gripper_position, gripper_quat)
        time.sleep(0.5)
        self.move_to_joints(pregrasp_joints)
        self.get_logger().info('pick 완료')


def main():
    if not rclpy.ok():
        rclpy.init()
    node = PickYoloTarget()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
