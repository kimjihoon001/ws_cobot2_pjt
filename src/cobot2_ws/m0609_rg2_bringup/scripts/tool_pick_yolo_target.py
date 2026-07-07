import math
import time
import cv2

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PlanningOptions, RobotState,
    PositionConstraint, OrientationConstraint, JointConstraint,
    CollisionObject, AttachedCollisionObject,
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from onrobot_rg_msgs.srv import SetCommand

MODEL_PATH = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/voice_processing/resource/toolbest.pt'
CLASS_NAMES = {0: 'screw2', 1: 'tool-hammer'}
TARGET_CLASS = 'tool-hammer'   # 잡을 대상. 나사면 'screw2'로 변경
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


def load_stl_mesh(filepath):
    mesh = Mesh()
    vertices_map = {}
    current_triangle = []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('vertex'):
                parts = line.split()
                # vertex x y z
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                
                # 이미 m 단위이므로 스케일링 불필요
                pt_key = (round(x, 6), round(y, 6), round(z, 6))
                if pt_key not in vertices_map:
                    p = Point()
                    p.x, p.y, p.z = x, y, z
                    vertices_map[pt_key] = len(mesh.vertices)
                    mesh.vertices.append(p)
                
                current_triangle.append(vertices_map[pt_key])
                if len(current_triangle) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = current_triangle
                    mesh.triangles.append(triangle)
                    current_triangle = []
    return mesh


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
            is_target = (best is not None and class_name == TARGET_CLASS and score >= CONFIDENCE_THRESHOLD)
            color = (0, 255, 0) if is_target else (128, 128, 128)  # 타겟=녹색, 기타=회색
            thickness = 3 if is_target else 1
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(vis, f'{class_name} {score:.2f}', (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)
        # 타겟 키포인트 그리기
        if best is not None:
            _, (bx1, by1, bx2, by2), bidx = best
            cx, cy = int((bx1 + bx2) / 2), int((by1 + by2) / 2)
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)  # 중심점 빨간 원
            role = KEYPOINT_AXIS_ROLE[TARGET_CLASS]
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
        cv2.putText(vis, f'Threshold: {CONFIDENCE_THRESHOLD:.0%}  Target: {TARGET_CLASS}',
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
        """color/depth/intrinsics가 갖춰지면 TARGET_CLASS 중 최고 confidence 탐지 1건을 반환."""
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
                if class_name != TARGET_CLASS:
                    continue
                if best is None or score > best[0]:
                    best = (score, box_xyxy, idx)

            # 시각화 (탐지 여부와 무관하게 매 프레임 갱신)
            self._visualize(self.color_frame, results, best)

            if best is None:
                continue

            score, (x1, y1, x2, y2), idx = best
            cx_px, cy_px = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_m = self._sample_depth(cx_px, cy_px)
            if depth_m is None:
                continue

            role = KEYPOINT_AXIS_ROLE[TARGET_CLASS]
            kpts_xy = results.keypoints.xy[idx].tolist()
            head_px = np.mean([kpts_xy[j] for j in role['head_idx']], axis=0)
            tail_px = np.array(kpts_xy[role['tail_idx']])

            self.get_logger().info(f'{TARGET_CLASS} 탐지 (conf={score:.2f}, depth={depth_m:.3f}m)')
            return cx_px, cy_px, depth_m, head_px, tail_px

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
        """head<->tail keypoint 축을 base_link 수평면 각도로 변환하고, 그리퍼가
        축과 90도로(손잡이 폭 방향) 물리도록 수직으로 돌린 yaw를 반환."""
        head_depth = self._sample_depth(int(round(head_px[0])), int(round(head_px[1])))
        tail_depth = self._sample_depth(int(round(tail_px[0])), int(round(tail_px[1])))
        if head_depth is None or tail_depth is None:
            self.get_logger().warn('축 keypoint depth 없음 — yaw=0(기본 top-down)으로 대체')
            return 0.0

        hx, hy, _ = self.pixel_to_base_xyz(int(round(head_px[0])), int(round(head_px[1])), head_depth)
        tx, ty, _ = self.pixel_to_base_xyz(int(round(tail_px[0])), int(round(tail_px[1])), tail_depth)
        axis_angle = math.atan2(hy - ty, hx - tx)
        yaw = axis_angle  # 해머 축과 수직 방향으로 잡기 위해 90도 돌린 각도 반영 (기존 +pi/2가 평행으로 동작했으므로 제거)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # -180~180도로 정규화

        # 평행 2핑거 그리퍼는 yaw와 yaw+180도*k가 전부 같은 파지 결과.
        # top-down 그립에서 yaw 변화는 대체로 joint_6가 흡수하므로(근사치, 정확한 IK
        # 값은 아님), 여러 후보 중 현재 joint_6와 가장 가까운 값을 선택해서
        # joint_6가 불필요하게 큰 회전/랩어라운드를 하지 않게 함.
        if self.current_joint6 is None:
            # 현재 값을 모르면 기본 -90~90도로 접어서 반환
            if yaw > math.pi / 2:
                yaw -= math.pi
            elif yaw <= -math.pi / 2:
                yaw += math.pi
            return yaw

        candidates = [yaw + k * math.pi for k in range(-3, 4)]
        return min(candidates, key=lambda c: abs(c - self.current_joint6))

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

    def spawn_attached_object(self, obj_id, mesh_msg, x, y, z, yaw, link_name='base_link'):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = obj_id
        
        pose = Pose()
        # yaw 방향으로 9cm(메쉬 길이 18cm의 절반)만큼 뒤로 밀어서, 검출된 중심점 (x,y,z)에 스크루드라이버의 기하학적 중심이 정렬되도록 오프셋 적용
        pose.position.x = x - 0.09 * math.cos(yaw)
        pose.position.y = y - 0.09 * math.sin(yaw)
        pose.position.z = z
        
        # 서 있는 스크루드라이버 STL 메쉬를 90도 피치 회전시켜 눕히고 yaw 적용
        qx, qy, qz, qw = quat_from_rpy(0.0, math.pi / 2, yaw)
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
            'rg2_base_link'
        ]
        
        for _ in range(5):
            self._attached_collision_pub.publish(aco)
            time.sleep(0.1)
        self.get_logger().info(f"물체 '{obj_id}'가 '{link_name}'에 부착된 상태(실제 장애물 판정)로 스폰되었습니다.")

    def attach_object_to_gripper(self, obj_id, mesh_msg):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        
        aco = AttachedCollisionObject()
        aco.link_name = EEF_LINK
        
        co = CollisionObject()
        co.header.frame_id = EEF_LINK  # 그리퍼 팁(rg2_tcp) 기준 좌표계
        co.id = obj_id
        
        pose = Pose()
        # 그리퍼 팁(rg2_tcp) 기준 로컬 좌표계 오프셋 설정
        # 18cm 스크루드라이버의 기하학적 중심이 rg2_tcp 중심에 오도록 로컬 Z축 기준 -9cm 오프셋 적용
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.09
        
        # 그리퍼 팁 방향과 일치하도록 메쉬를 눕혀둠 (로컬 피치 90도)
        qx, qy, qz, qw = quat_from_rpy(0.0, math.pi / 2, 0.0)
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
            'rg2_base_link'
        ]
        
        for _ in range(5):
            self._attached_collision_pub.publish(aco)
            time.sleep(0.1)
        self.get_logger().info(f"물체 '{obj_id}' 그리퍼에 부착 완료")
    def run(self):
        self.get_logger().info(f'{TARGET_CLASS} 탐지 대기중...')
        detection = self.detect_once()
        if detection is None:
            self.get_logger().error('타겟을 찾지 못함 (타임아웃)')
            return

        cx, cy, depth_m, head_px, tail_px = detection
        x, y, z = self.pixel_to_base_xyz(cx, cy, depth_m)
        yaw = self.axis_yaw(head_px, tail_px)
        self.get_logger().info(
            f'base_link 좌표: ({x:.3f}, {y:.3f}, {z:.3f}), yaw={math.degrees(yaw):.1f}deg')

        # 해머 검출 위치에 screwdriver.stl 메쉬 스폰 (base_link에 부착된 실제 장애물 상태)
        stl_path = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup/meshes/screwdriver.stl'
        try:
            mesh_msg = load_stl_mesh(stl_path)
            self.spawn_attached_object('hammer', mesh_msg, x, y, z, yaw)
            self.get_logger().info('해머 위치에 STL 스폰 완료')
        except Exception as e:
            self.get_logger().error(f'STL 스폰 실패: {e}')

        pregrasp = self.build_pose(x, y, z + PREGRASP_Z_OFFSET, yaw)
        grasp = self.build_pose(x, y, z + GRASP_Z_CLEARANCE, yaw)

        # 관절 공간 제어(Joint Space Control)를 위한 IK 풀이
        pregrasp_joints = self.solve_ik(x, y, z + PREGRASP_Z_OFFSET, yaw)
        grasp_joints = self.solve_ik(x, y, z + GRASP_Z_CLEARANCE, yaw, seed_joints=pregrasp_joints)
 
        if pregrasp_joints is None or grasp_joints is None:
            self.get_logger().error('pregrasp 또는 grasp 포즈에 대한 IK 해를 찾지 못했습니다. 기동을 취소합니다.')
            return

        if not self.gripper_command('o'):
            return
        if not self.move_to_joints(pregrasp_joints):
            return
        if not self.move_to_joints(grasp_joints):
            return
        if not self.gripper_command('c'):
            return
        # 그리퍼를 닫은 후, 월드(base_link)에 있던 장애물을 그리퍼 프레임으로 리어태치
        self.attach_object_to_gripper('hammer', mesh_msg)
        time.sleep(0.5)
        self.move_to_joints(pregrasp_joints)
        self.get_logger().info('pick 완료')


def main():
    rclpy.init()
    node = PickYoloTarget()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
