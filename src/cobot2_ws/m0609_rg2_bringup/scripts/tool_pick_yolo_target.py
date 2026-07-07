import math
import time

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PlanningOptions, RobotState,
    PositionConstraint, OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from onrobot_rg_msgs.srv import SetCommand

MODEL_PATH = '/home/rokey/ws_cobot2_pjt/src/cobot2_ws/voice_processing/resource/toolbest.pt'
CLASS_NAMES = {0: 'screw2', 1: 'tool-hammer'}
TARGET_CLASS = 'tool-hammer'   # 잡을 대상. 나사면 'screw2'로 변경
CONFIDENCE_THRESHOLD = 0.5
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
USE_GRIPPER = False   # 그리퍼 미연결 시 False (연결 시 True로 변경)

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

        self.create_subscription(Image, '/camera/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

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
        self.get_logger().info('준비 완료')

    def _color_cb(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.camera_frame_id = msg.header.frame_id

    def _depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _camera_info_cb(self, msg):
        self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'ppx': msg.k[2], 'ppy': msg.k[5]}

    def _joint_state_cb(self, msg):
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

        transform = self.tf_buffer.lookup_transform(
            PLANNING_FRAME, self.camera_frame_id, rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
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
        yaw = axis_angle + math.pi / 2
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

        pregrasp = self.build_pose(x, y, z + PREGRASP_Z_OFFSET, yaw)
        grasp = self.build_pose(x, y, z + GRASP_Z_CLEARANCE, yaw)

        if not self.gripper_command('o'):
            return
        if not self.move_to_pose(pregrasp):
            return
        if not self.move_to_pose(grasp):
            return
        if not self.gripper_command('c'):
            return
        time.sleep(0.5)
        self.move_to_pose(pregrasp)
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
