import os
import sys
import time
import json
import sqlite3
import threading

import numpy as np
import cv2
from cv_bridge import CvBridge
import trimesh
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
try:
    from ultralytics import YOLO
except ImportError:
    pass

from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import String
from od_msg.srv import SrvDepthPosition
from robot_control.onrobot import RG

from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, PointStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
import tf2_geometry_msgs
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState
)
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from dsr_msgs2.srv import MoveStop

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Initialize DSR Node
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40  # Slower for safety during inspection
SCAN_DISTANCE = 250.0   # mm from Jenga block center
SCAN_HEIGHT = 100.0     # mm height offset above Jenga center
CAMERA_Y_OFFSET = 75.0  # mm offset of camera from TCP

# Jenga Tower Dimensions ( 6 layers: 75mm x 75mm x 84mm )
JENGA_WIDTH = 75.0   # mm (3 blocks * 25mm = 75mm)
JENGA_DEPTH = 75.0   # mm (75mm)
JENGA_HEIGHT = 84.0  # mm (6 layers * 14mm = 84mm)

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

if not rclpy.ok():
    rclpy.init()

dsr_node = rclpy.create_node("jenga_inspector_dsr_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movejx, movel, mwait, get_current_posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Define expected configuration for PASS/FAIL
# Face 0: Front, Face 1: Right, Face 2: Back, Face 3: Left
# You can customize these expected hole counts for your Jenga assembly.
EXPECTED_FEATURES = {
    0: {"smallhole": 2, "longhole": 1},
    1: {"smallhole": 1, "longhole": 2},
    2: {"smallhole": 2, "longhole": 1},
    3: {"smallhole": 1, "longhole": 2}
}

# 컨베이어(conveyor_serial/conveyor_control.py, 저장소 루트) 연동 설정.
# MOVE는 상대 이동이라 7700 -> 검사장소, 합격 시 추가로 2300 -> 누적 10000 지점.
# 검사장소 이동 후에는 로봇팔이 바로 검사를 시작해야 해서 완료를 기다리지만(펌웨어가
# 이동 완료를 시리얼로 알려주지 않아 실측 소요시간+여유로 sleep), 합격 후 2300 이동은
# 그 뒤에 로봇팔이 할 일이 없어서 완료를 기다리지 않고 그냥 보내기만 한다.
CONVEYOR_MOVE_TO_INSPECTION_STEPS = -7700
CONVEYOR_MOVE_TO_INSPECTION_WAIT_SEC = 28.0    # 실측 26.94초 + 여유
CONVEYOR_MOVE_TO_PASS_STEPS = -2300

# 불합격 시 젠가 블록을 밀어내는 동작 (moveit_joint_line_demo.py의 두 자세, 단위: deg)
# PUSH_APPROACH_JOINTS_DEG로 접근한 뒤 PUSH_TARGET_JOINTS_DEG로 이동하며 민다.
PUSH_APPROACH_JOINTS_DEG = [-39.51, -21.94, 115.68, -0.33, 85.92, 140.73]
PUSH_TARGET_JOINTS_DEG = [-67.14, 10.59, 87.35, -0.11, 81.50, 113.21]
DOOSAN_QUICK_STOP_MODE = 1  # DR_QSTOP: Quick stop without STO
DOOSAN_MOVE_STOP_SERVICE = f'/{ROBOT_ID}/motion/move_stop'


class JengaInspectorNode(Node):
    def __init__(self):
        super().__init__("jenga_inspector")
        self.package_path = get_package_share_directory("robot_control")
        self.init_database()
        
        # OnRobot RG2 Gripper setup
        self.gripper = RG("rg2", "192.168.1.1", "502")

        # YOLO init
        possible_ws_paths = [
            os.path.expanduser("~/ws_cobot2_pjt"),
            os.path.expanduser("~/cobot_ws/src/ws_cobot2_pjt")
        ]

        workspace_path = possible_ws_paths[0]
        for path in possible_ws_paths:
            if os.path.exists(os.path.join(path, 'src/yolov8_ws/model/best_3.onnx')):
                workspace_path = path
                break

        # 컨베이어 제어(conveyor_serial/conveyor_control.py)는 ROS 패키지가 아니라
        # 저장소 루트(workspace_path)에 있는 일반 파이썬 모듈이라 sys.path에 추가해서 가져온다.
        # 시리얼 포트가 없거나(케이블 미연결 등) 열기 실패해도 검사 자체는 계속 되도록
        # 예외를 여기서 잡고 self.conveyor = None으로 둔다.
        self.conveyor = None
        try:
            conveyor_dir = os.path.join(workspace_path, 'conveyor_serial')
            if conveyor_dir not in sys.path:
                sys.path.insert(0, conveyor_dir)
            from conveyor_control import ConveyorController
            self.conveyor = ConveyorController()
            self.get_logger().info("컨베이어 시리얼 연결 완료.")
        except Exception as e:
            self.get_logger().error(f"컨베이어 시리얼 연결 실패 - 컨베이어 이동 없이 진행: {e}")

        model_path = os.path.join(workspace_path, 'src/yolov8_ws/model/best_3.onnx')
        if not os.path.exists(model_path):
            model_path = os.path.join(workspace_path, 'src/yolov8_ws/model/best_2.onnx')
        self.model = YOLO(model_path, task='detect')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.depth_frame = None
        self.intrinsics = None
        self.inspection_data = {}
        self.current_pitch_deg = 45.0
        
        self.REFERENCE_TEMPLATES = {
            "기본 완제품 (누락 없음)": (
                "OOO", "OOO", "OOO", "OOO", "OOO", "OOO"
            ),
            "A형 상품 (3층 중앙 누락)": (
                "OOO", "OOO", "OXO", "OOO", "OOO", "OOO"
            ),
            "B형 상품 (4층 양끝 누락)": (
                "OOO", "OOO", "OOO", "XOX", "OOO", "OOO"
            ),
            "C형 상품 (2층 중앙, 4층 중앙 누락)": (
                "OOO", "OXO", "OOO", "OXO", "OOO", "OOO"
            )
        }

        from rclpy.callback_groups import ReentrantCallbackGroup
        cb_group = ReentrantCallbackGroup()
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10, callback_group=cb_group)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10, callback_group=cb_group)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10, callback_group=cb_group)
        self.image_pub = self.create_publisher(Image, '/yolo/result_image', 10)

        # Helper node for blocking action and service calls to prevent executor deadlock
        self.action_node = rclpy.create_node("jenga_inspector_action_node")
        self.ikin_client = self.action_node.create_client(GetPositionIK, '/compute_ik')
        self._ac = ActionClient(self.action_node, MoveGroup, '/move_action')
        self.move_stop_service_name = DOOSAN_MOVE_STOP_SERVICE
        self.move_stop_client = self.action_node.create_client(
            MoveStop,
            self.move_stop_service_name
        )

        # Clients for perception (created on action_node to bypass executor deadlock)
        self.get_position_client = self.action_node.create_client(
            SrvDepthPosition, "/get_jenga_position"
        )
        self.detect_features_client = self.action_node.create_client(
            Trigger, "/detect_jenga_features"
        )
        
        self.action_executor = rclpy.executors.SingleThreadedExecutor()
        self.action_executor.add_node(self.action_node)
        self.action_thread = threading.Thread(target=self.action_executor.spin, daemon=True)
        self.action_thread.start()

        self.get_logger().info("Waiting for MoveGroup Action server...")
        self._ac.wait_for_server()
        self.get_logger().info("MoveGroup Action server connected.")
        if self.move_stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f"Doosan move_stop service connected: {self.move_stop_service_name}")
        else:
            self.get_logger().warn(
                f"Doosan move_stop service not available yet: {self.move_stop_service_name}"
            )

        # TF Listener to get current robot pose without blocking DSR services
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for RViz Planning Scene (MoveIt)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Service for triggering inspection
        self.create_service(
            Trigger,
            '/run_jenga_inspection',
            self.handle_run_jenga_inspection
        )
        
        # Hand avoidance parameters & states
        self.hand_detected = False
        self.current_hand_pos = None
        self.active_goal_handle = None
        self.hmi_stop_requested = False
        self.in_evasion = False
        self.in_push_sequence = False
        self.push_hand_seen = False
        self.push_cancel_requested = False
        self.last_moveit_error_code = None
        self.last_log_time = 0.0
        
        # Subscribe to hand position topic with a ReentrantCallbackGroup to prevent service call deadlocks
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.hand_sub = self.create_subscription(
            Point,
            '/hand_position',
            self.hand_position_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        self.hmi_stop_sub = self.create_subscription(
            String,
            '/hmi/emergency_stop',
            self.hmi_stop_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("JengaInspectorNode initialized. Service '/run_jenga_inspection' is ready.")

    def hmi_stop_callback(self, msg):
        if msg.data == "release":
            self.hmi_stop_requested = False
            self.get_logger().info("HMI 긴급정지 해제 신호 수신.")
            return
        self.hmi_stop_requested = True
        self.get_logger().warn("HMI 긴급정지 신호 수신 - 현재 MoveGroup goal 취소 요청.")
        goal_handle = self.active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f"HMI 긴급정지 goal 취소 요청 실패: {e}")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            pass

    def get_vertical_depth(self, u, v, Z_c, pitch_deg=45.0):
        if Z_c <= 0 or self.intrinsics is None:
            return None
        pitch = np.radians(pitch_deg)
        fx = self.intrinsics['fx']
        ppx = self.intrinsics['ppx']
        
        # 젠가 꼭대기가 왼쪽(u=0), 바닥이 오른쪽(u=max)인 상황: u가 커질수록 아래로 이동
        X_c = (u - ppx) * Z_c / fx
        
        # Z_c는 카메라의 깊이, X_c는 이미지 상의 가로(실제로는 젠가의 세로) 방향
        # pitch=0은 꼭대기에서 수직으로 내려다보는 상태
        return Z_c * np.cos(pitch) + X_c * np.sin(pitch)

    def capture_single_frame(self, face_name, pitch_deg):
        self.get_logger().info(f"[{face_name}] 각도 {pitch_deg}°에서 1프레임 캡처 중...")
        if self.latest_image is None or self.depth_frame is None or self.intrinsics is None:
            self.get_logger().warn("카메라 데이터를 기다리는 중...")
            time.sleep(0.5)
            return []
        
        frame_results = []
        img_copy = self.latest_image.copy()
        results = self.model(img_copy, conf=0.60, verbose=False)
        
        if len(results) > 0:
            res = results[0]
            annotated_frame = res.plot()
            
            # 이미지 저장 (백엔드 static 폴더)
            possible_paths = [
                os.path.expanduser("~/ws_cobot2_pjt/backend"),
                os.path.expanduser("~/cobot_ws/src/ws_cobot2_pjt/backend")
            ]
            db_dir = possible_paths[0]
            for p in possible_paths:
                if os.path.exists(p):
                    db_dir = p
                    break
            
            save_dir = os.path.join(db_dir, "static", "inspection_images")
            os.makedirs(save_dir, exist_ok=True)
            timestamp = int(time.time() * 1000)
            img_filename = f"inspection_{face_name}_{pitch_deg}_{timestamp}.jpg"
            filename = os.path.join(save_dir, img_filename)
            image_saved = cv2.imwrite(filename, annotated_frame)
            if not image_saved:
                self.get_logger().error(f"검출 이미지 저장 실패: {filename}")
            
            if image_saved and abs(pitch_deg - 45.0) < 1.0:
                self.current_inspection_images.append(f"/static/inspection_images/{img_filename}")
            
            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(img_msg)
            except Exception:
                pass
            
            names = res.names
            entire_box = None
            hole_boxes = []
            
            for box in res.boxes:
                cls_name = names[int(box.cls[0])]
                if cls_name == 'entire':
                    entire_box = box
                elif cls_name.lower() == 'smallhole':
                    hole_boxes.append(box)
            
            if entire_box is not None:
                ex1, ey1, ex2, ey2 = entire_box.xyxy[0].cpu().numpy()
                eu = int((ex1 + ex2) / 2)
                ev = int((ey1 + ey2) / 2)
                
                valid_entire_Z = float('inf')
                valid_eu, valid_ev = eu, ev
                
                scan_ey_start = int(ey1) + 10
                scan_ey_end = int(ey2) - 10
                
                for scan_v in range(scan_ey_start, scan_ey_end + 1, 5):
                    tu = np.clip(eu, 0, self.depth_frame.shape[1]-1)
                    tv = np.clip(scan_v, 0, self.depth_frame.shape[0]-1)
                    val = float(self.depth_frame[tv, tu])
                    if 0 < val < valid_entire_Z:
                        valid_entire_Z = val
                        valid_eu, valid_ev = tu, tv
                
                if valid_entire_Z != float('inf'):
                    entire_v_depth = self.get_vertical_depth(valid_eu, valid_ev, valid_entire_Z, pitch_deg=pitch_deg)
                    if entire_v_depth is not None:
                        pitch = np.radians(pitch_deg)
                        fx = self.intrinsics['fx']
                        ppx = self.intrinsics['ppx']
                        
                        X_ent_norm = (valid_eu - ppx) / fx
                        K_horiz = valid_entire_Z * (np.sin(pitch) - X_ent_norm * np.cos(pitch))
                        
                        for box in hole_boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            u = int((x1 + x2) / 2)
                            v = int((y1 + y2) / 2)
                            
                            X_hole_norm = (u - ppx) / fx
                            denom = np.sin(pitch) - X_hole_norm * np.cos(pitch)
                            if abs(denom) > 1e-6:
                                Z_hole_true = K_horiz / denom
                                obj_v_depth = self.get_vertical_depth(u, v, Z_hole_true, pitch_deg=pitch_deg)
                                if obj_v_depth is not None:
                                    # 84mm 젠가 타워의 정확한 중앙(entire_v_depth)을 기준점(42mm)으로 삼아 높이 역산
                                    # 바운딩 박스(ex1)에 의존하지 않아 그림자 노이즈에 매우 강건함
                                    height_from_base = 42.0 + (entire_v_depth - obj_v_depth)
                                    compensated_height = height_from_base - 7.0
                                    floor_num = max(1, int(round(compensated_height / 14.0)) + 1)
                                    floor_num = min(6, floor_num)
                                    
                                    tower_width = ey2 - ey1
                                    third = tower_width / 3.0
                                    if v < ey1 + third:
                                        horiz_pos = "Left"
                                    elif v < ey1 + 2 * third:
                                        horiz_pos = "Center"
                                    else:
                                        horiz_pos = "Right"
                                        
                                    frame_results.append((floor_num, horiz_pos))
        return frame_results

    def analyze_defects(self):
        self.get_logger().info("===== 최종 검사 및 패턴 매칭 =====")
        jenga_map = {floor: ["O", "O", "O"] for floor in range(6, 0, -1)}
        
        pos_to_idx = {"Left": 0, "Center": 1, "Right": 2}
        mirror_pos = {"Left": "Right", "Center": "Center", "Right": "Left"}
        
        # 방향에 상관없이 YOLO가 찾아낸 구멍을 거울 반전(Mirror) 없이 그대로 맵에 반영합니다.
        # (양품 템플릿이 좌우 대칭이므로, 보는 방향에 따라 좌우가 뒤바뀌어도 무방하다는 사용자 요청)
        for face_id, data in self.inspection_data.items():
            for floor, pos in data:
                if 1 <= floor <= 6:
                    jenga_map[floor][pos_to_idx[pos]] = "X"


        print("\n================ Jenga 6-Floor Map ================")
        print("  [Floor]  [Left]  [Center]  [Right]")
        for floor in range(6, 0, -1):
            print(f"  Floor {floor}:   {jenga_map[floor][0]}        {jenga_map[floor][1]}         {jenga_map[floor][2]}")
        print("===================================================\n")
        
        # 1층부터 6층까지 튜플 생성
        current_map_list = []
        for floor in range(1, 7):
            current_map_list.append("".join(jenga_map[floor]))
        current_map_tuple = tuple(current_map_list)
        
        matched_product = None
        for product_name, template in self.REFERENCE_TEMPLATES.items():
            if current_map_tuple == template:
                matched_product = product_name
                break
                
        if matched_product:
            product_type = matched_product
            is_pass = True
            self.final_matched_product = matched_product
        else:
            product_type = "불량품 (알 수 없는 패턴)"
            is_pass = False
            self.final_matched_product = "FAIL"
            
        print(f"-> 최종 판정 결과: {product_type}")
        self.final_jenga_map = jenga_map
        return is_pass

    def hand_position_callback(self, msg):
        """Processes hand position, filters by robot workspace ROI, and updates obstacle."""
        is_clear_signal = msg.z < -1.0
        
        # 기존에 ROI가 좁아서 손이 경계에 있을 때 스크립트는 무시하고 MoveIt만 피해서 가는 문제 발생.
        # 작업 공간 앞쪽 전체를 커버하도록 ROI를 대폭 확장합니다.
        # X: 0cm ~ 80cm, Y: -60cm ~ 60cm, Z: -20cm ~ 80cm
        in_roi = False
        if not is_clear_signal:
            in_roi = (0.0 <= msg.x <= 0.80) and (-0.60 <= msg.y <= 0.60) and (-0.20 <= msg.z <= 0.80)
            
        if is_clear_signal or not in_roi:
            if self.hand_detected:
                self.hand_detected = False
                self.current_hand_pos = None
                self.clear_hand_obstacle()
                self.get_logger().info("Hand cleared (left FOV or exited workspace ROI).")
        else:
            self.hand_detected = True
            self.current_hand_pos = [msg.x, msg.y, msg.z]
            now = time.time()
            if now - self.last_log_time > 1.0:
                self.get_logger().info(f"Hand detected inside active ROI: x={msg.x:.3f}m, y={msg.y:.3f}m, z={msg.z:.3f}m")
                self.last_log_time = now
            if getattr(self, 'in_push_sequence', False):
                self.push_hand_seen = True
                if not self.push_cancel_requested:
                    self.push_cancel_requested = True
                    self.request_doosan_quick_stop("밀기 중 손 감지")
                    goal_handle = self.active_goal_handle
                    if goal_handle is not None:
                        try:
                            goal_handle.cancel_goal_async()
                            self.get_logger().warn("밀기 중 손 감지 - 현재 MoveGroup goal 즉시 취소 요청.")
                        except Exception as e:
                            self.get_logger().error(f"밀기 중 MoveGroup goal 취소 요청 실패: {e}")
            else:
                self.update_hand_obstacle(msg.x, msg.y, msg.z)

    def call_clear_octomap(self):
        """Clears MoveIt's Octomap to force straight planning without avoidance."""
        client = self.create_client(Empty, '/clear_octomap')
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('/clear_octomap service not available')
            return
        req = Empty.Request()
        client.call_async(req)
        self.get_logger().info('Cleared octomap to force straight planning.')

    def request_doosan_quick_stop(self, reason):
        """Requests a controller-level quick stop; MoveGroup cancel alone can lag behind execution."""
        if not self.ensure_move_stop_service_ready():
            self.get_logger().error(
                f"{reason} - Doosan quick stop 서비스가 준비되지 않아 로봇 정지 요청을 보내지 못했습니다."
            )
            return

        req = MoveStop.Request()
        req.stop_mode = DOOSAN_QUICK_STOP_MODE

        try:
            future = self.move_stop_client.call_async(req)
            future.add_done_callback(self._on_doosan_quick_stop_done)
            self.get_logger().warn(f"{reason} - Doosan quick stop 요청 전송.")
        except Exception as e:
            self.get_logger().error(f"{reason} - Doosan quick stop 요청 실패: {e}")

    def ensure_move_stop_service_ready(self):
        if self.move_stop_client.service_is_ready():
            return True

        if self.move_stop_client.wait_for_service(timeout_sec=0.5):
            return True

        try:
            service_names = [name for name, _ in self.action_node.get_service_names_and_types()]
            candidates = sorted(name for name in service_names if name.endswith('/motion/move_stop'))
            if candidates and candidates[0] != self.move_stop_service_name:
                self.move_stop_service_name = candidates[0]
                self.move_stop_client = self.action_node.create_client(MoveStop, self.move_stop_service_name)
                self.get_logger().warn(f"Doosan move_stop service 재연결 시도: {self.move_stop_service_name}")
        except Exception as e:
            self.get_logger().warn(f"Doosan move_stop service 검색 실패: {e}")

        return self.move_stop_client.wait_for_service(timeout_sec=1.0)

    def _on_doosan_quick_stop_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Doosan quick stop 응답 실패: {e}")
            return

        if result and result.success:
            self.get_logger().warn("Doosan quick stop 처리 완료.")
        else:
            self.get_logger().error("Doosan quick stop 처리 실패.")

    def update_hand_obstacle(self, x, y, z):
        """Spawns/Updates hand obstacle in the MoveIt planning scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'human_hand'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.025] # 2.5cm safety sphere radius (5cm diameter)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives = [sphere]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
        self.scene_pub.publish(scene)

    def clear_hand_obstacle(self):
        """Removes the hand obstacle from the MoveIt planning scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'human_hand'
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
        self.scene_pub.publish(scene)
        self.get_logger().info("Cleared hand obstacle from planning scene.")

    def init_database(self):
        """Connects to the backend SQLite database."""
        # 백엔드의 cobot.db 경로를 찾습니다 (노트북 환경마다 다를 수 있음)
        possible_paths = [
            os.path.expanduser("~/ws_cobot2_pjt/backend"),
            os.path.expanduser("~/cobot_ws/src/ws_cobot2_pjt/backend")
        ]
        
        db_dir = possible_paths[0]
        for path in possible_paths:
            if os.path.exists(path):
                db_dir = path
                break
                
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)
            
        self.db_path = os.path.join(db_dir, "cobot.db")
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # 1. inspection_results 테이블 생성
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS inspection_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                product VARCHAR(100) NOT NULL,
                result VARCHAR(10) NOT NULL,
                defect_location VARCHAR(100),
                map_data TEXT,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # 2. resources 테이블 생성 (백엔드 미가동 시 OperationalError 방지)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS resources (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name VARCHAR(100) NOT NULL,
                item_type VARCHAR(20) NOT NULL DEFAULT 'material',
                category VARCHAR(50) NOT NULL DEFAULT '',
                quantity INTEGER NOT NULL DEFAULT 0,
                unit VARCHAR(20) NOT NULL DEFAULT 'EA',
                min_quantity INTEGER NOT NULL DEFAULT 0,
                location VARCHAR(100) NOT NULL DEFAULT '',
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # 3. inventory_logs 테이블 생성 (백엔드 미가동 시 OperationalError 방지)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS inventory_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                resource_name VARCHAR(100) NOT NULL,
                action VARCHAR(20) NOT NULL,
                detail VARCHAR(255) NOT NULL DEFAULT '',
                username VARCHAR(50) NOT NULL,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        conn.commit()
        conn.close()
        self.get_logger().info(f"Database initialized at {self.db_path}")

    def solve_ik(self, pose):
        """Solves inverse kinematics for a Cartesian pose using MoveIt compute_ik.
        Accepts [x,y,z,rx,ry,rz] (Euler ZYZ) or [x,y,z,qx,qy,qz,qw] (quaternion).
        Returns joint angles in degrees, or None if failed."""
        if not self.ikin_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK service /compute_ik not available")
            return None
            
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'manipulator'
        ik_req.ik_link_name = 'tool0' # Solve IK for tool0 flange
        ik_req.pose_stamped.header.frame_id = 'base_link'
        
        # Position in meters
        ik_req.pose_stamped.pose.position.x = pose[0] / 1000.0
        ik_req.pose_stamped.pose.position.y = pose[1] / 1000.0
        ik_req.pose_stamped.pose.position.z = pose[2] / 1000.0
        
        if len(pose) == 7:
            # Pose with quaternion orientation
            ik_req.pose_stamped.pose.orientation.x = pose[3]
            ik_req.pose_stamped.pose.orientation.y = pose[4]
            ik_req.pose_stamped.pose.orientation.z = pose[5]
            ik_req.pose_stamped.pose.orientation.w = pose[6]
        else:
            # Pose with Euler angles ZYZ in degrees
            r = Rotation.from_euler('ZYZ', [pose[3], pose[4], pose[5]], degrees=True)
            q = r.as_quat()
            ik_req.pose_stamped.pose.orientation.x = q[0]
            ik_req.pose_stamped.pose.orientation.y = q[1]
            ik_req.pose_stamped.pose.orientation.z = q[2]
            ik_req.pose_stamped.pose.orientation.w = q[3]
            
        # Seed the solver using the initial pose (in radians) to avoid local minima or singularities
        joint_state = JointState()
        joint_state.name = JOINT_NAMES
        joint_state.position = [np.radians(angle) for angle in [-44.26, 18.14, 60.38, -0.02, 101.41, -36.57]]
        
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        ik_req.robot_state = robot_state
        
        ik_req.avoid_collisions = True
        req.ik_request = ik_req
        
        future = self.ikin_client.call_async(req)
        # Wait for the future to finish without blocking the ROS 2 spin loop
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            
        res = future.result()
        # MoveIt GetPositionIK returns error_code.val == 1 for success
        if res and res.error_code.val == 1:
            joint_names = res.solution.joint_state.name
            joint_positions_rad = res.solution.joint_state.position
            joint_positions_deg = []
            for name in JOINT_NAMES:
                try:
                    idx = joint_names.index(name)
                    val_rad = joint_positions_rad[idx]
                    joint_positions_deg.append(np.degrees(val_rad))
                except ValueError:
                    self.get_logger().error(f"Joint {name} not found in IK solution!")
                    return None
            return joint_positions_deg
        else:
            err_val = res.error_code.val if res else "No response"
            self.get_logger().error(f"MoveIt IK solution failed for pose: {pose}. Error code: {err_val}")
            return None

    def get_current_pose_tf(self):
        """Gets current pose of tool0 relative to base_link using TF.
        Returns [x, y, z, qx, qy, qz, qw] where positions are in mm and orientation is quaternion."""
        try:
            now = rclpy.time.Time()
            # Wait up to 1.0s for the transform
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Position in mm
            x = trans.transform.translation.x * 1000.0
            y = trans.transform.translation.y * 1000.0
            z = trans.transform.translation.z * 1000.0
            
            # Orientation quaternion
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            return [x, y, z, qx, qy, qz, qw]
        except Exception as e:
            self.get_logger().error(f"Failed to lookup robot pose via TF: {e}")
            # Fallback to get_current_posx
            try:
                self.get_logger().info("Attempting fallback to get_current_posx...")
                posx, _ = get_current_posx()
                # Convert Euler ZYZ to quaternion
                r = Rotation.from_euler('ZYZ', [posx[3], posx[4], posx[5]], degrees=True)
                q = r.as_quat()
                return [posx[0], posx[1], posx[2], q[0], q[1], q[2], q[3]]
            except Exception as ex:
                self.get_logger().error(f"Fallback get_current_posx also failed: {ex}")
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    def move_to_joints_moveit(self, joint_positions_deg):
        """Plans and executes motion to the target joint positions (in degrees) using MoveGroup action.
        Aborts planning or execution if a hand is detected (unless in safety evasion mode)."""
        self.last_moveit_error_code = None
        hand_blocks_motion = lambda: self.hand_detected or (
            getattr(self, 'in_push_sequence', False)
            and (getattr(self, 'push_hand_seen', False) or getattr(self, 'push_cancel_requested', False))
        ) or self.hmi_stop_requested

        # Convert degrees to radians
        joint_positions_rad = [np.radians(angle) for angle in joint_positions_deg]
        
        # Build joint constraints
        goal_constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, joint_positions_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'manipulator'
        req.start_state = RobotState()
        req.start_state.is_diff = True  # Start from current actual robot state
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 5.0
        # Velocity and acceleration scaling factors
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        opts = PlanningOptions()
        opts.plan_only = False  # Plan and execute immediately

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        # Check before sending goal (bypass check if in safety evasion mode)
        if hand_blocks_motion() and not getattr(self, 'in_evasion', False):
            self.last_moveit_error_code = 'HMI_STOP' if self.hmi_stop_requested else 'HAND_DETECTED'
            self.get_logger().warn('정지 조건 감지: planning 전 이동을 중단합니다.')
            return False

        self.get_logger().info('Sending MoveGroup Goal...')
        send_future = self._ac.send_goal_async(goal_msg)
        cancel_after_accept = False
        
        while rclpy.ok() and not send_future.done():
            if hand_blocks_motion() and not getattr(self, 'in_evasion', False):
                self.last_moveit_error_code = 'HMI_STOP' if self.hmi_stop_requested else 'HAND_DETECTED'
                cancel_after_accept = True
                self.get_logger().warn('정지 조건 감지: MoveGroup goal 수락 즉시 취소합니다.')
                if getattr(self, 'in_push_sequence', False):
                    self.request_doosan_quick_stop("밀기 goal 전송 중 손 감지")
                break
            time.sleep(0.05)

        while rclpy.ok() and not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.last_moveit_error_code = 'GOAL_REJECTED'
            self.get_logger().error('Motion planning request rejected by MoveGroup.')
            return False

        self.active_goal_handle = goal_handle

        if cancel_after_accept:
            cancel_future = goal_handle.cancel_goal_async()
            while rclpy.ok() and not cancel_future.done():
                time.sleep(0.01)
            self.active_goal_handle = None
            self.get_logger().info('Motion cancelled before execution due to hand detection.')
            return False

        self.get_logger().info('Motion planning accepted. Executing...')
        result_future = goal_handle.get_result_async()
        
        aborted = False
        while rclpy.ok() and not result_future.done():
            if hand_blocks_motion() and not getattr(self, 'in_evasion', False):
                self.get_logger().warn('정지 조건 감지: 현재 MoveGroup 실행을 취소합니다.')
                if getattr(self, 'in_push_sequence', False):
                    self.request_doosan_quick_stop("밀기 실행 중 손 감지")
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    time.sleep(0.01)
                self.get_logger().info('MoveGroup cancel request completed. Waiting 1.0s for robot to settle...')
                time.sleep(1.0)
                aborted = True
                break
            time.sleep(0.05)

        self.active_goal_handle = None
        
        if aborted:
            self.last_moveit_error_code = 'HMI_STOP' if self.hmi_stop_requested else 'HAND_DETECTED'
            return False

        result = result_future.result().result
        self.last_moveit_error_code = result.error_code.val
        if result.error_code.val == 1:
            self.get_logger().info('Successfully moved to target joint positions!')
            return True
        else:
            self.get_logger().error(f'MoveGroup execution failed with error code: {result.error_code.val}')
            return False

    def recover_to_push_approach(self):
        """밀기 중 취소 후 홈이 아니라 밀기 1 위치(PUSH_APPROACH_JOINTS_DEG)로 복귀한다."""
        if self.hmi_stop_requested:
            self.get_logger().warn("HMI 긴급정지 상태라 밀기 1 위치 복귀 명령을 보내지 않습니다.")
            return False

        self.get_logger().warn("밀기 동작 취소됨 - 현재 동작 정지 후 밀기 1 위치로 복귀합니다.")
        self.clear_hand_obstacle()
        self.call_clear_octomap()
        time.sleep(0.1)

        previous_evasion = self.in_evasion
        self.in_evasion = True
        try:
            return self.move_to_joints_moveit(PUSH_APPROACH_JOINTS_DEG)
        finally:
            self.in_evasion = previous_evasion

    def execute_safety_evasion(self):
        """Safely retreats to home position (JReady) by avoiding the hand obstacle.
        If planning fails, it stops and waits until the path is clear."""
        JReady = [-44.26, 18.14, 60.38, -0.02, 101.41, -36.57]
        self.get_logger().warn("Safety Evasion triggered! Attempting to move to Home (JReady) by avoiding obstacles...")
        
        self.in_evasion = True
        while rclpy.ok():
            success = self.move_to_joints_moveit(JReady)
            if success:
                self.get_logger().info("Safety Evasion complete. Successfully returned to Home (JReady).")
                break
            else:
                self.get_logger().warn("Safety Evasion planning failed (hand might be directly blocking). Retrying in 2.0 seconds...")
                time.sleep(2.0)
                
        self.in_evasion = False

    def log_result_to_db(self, product, result, defect_location, map_data_json, image_paths=None):
        import json
        if image_paths and len(image_paths) > 0:
            try:
                map_dict = json.loads(map_data_json)
                map_dict["images"] = image_paths
                map_data_json = json.dumps(map_dict)
            except Exception:
                pass
        """Logs inspection result to the database."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO inspection_results (product, result, defect_location, map_data, created_at) VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP)",
            (product, result.lower(), defect_location, map_data_json)
        )
        
        # 정상 제품(PASS)일 경우 재고(resources) 추가 및 로그 기록
        if result.lower() == 'pass':
            # 해당 상품명으로 재고가 있는지 확인
            cursor.execute("SELECT quantity FROM resources WHERE name = ?", (product,))
            row = cursor.fetchone()
            
            if row:
                # 이미 있으면 수량 + 1
                cursor.execute("UPDATE resources SET quantity = quantity + 1, updated_at = CURRENT_TIMESTAMP WHERE name = ?", (product,))
            else:
                # 없으면 새로 등록
                category_name = '기본형'
                if 'A형' in product: category_name = 'A형'
                elif 'B형' in product: category_name = 'B형'
                elif 'C형' in product: category_name = 'C형'
                
                cursor.execute(
                    "INSERT INTO resources (name, item_type, category, quantity, unit, min_quantity, location, created_at, updated_at) VALUES (?, 'product', ?, 1, 'EA', 0, '출하 대기장', CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)",
                    (product, category_name)
                )
                
            # 인벤토리 로그 남기기
            cursor.execute(
                "INSERT INTO inventory_logs (resource_name, action, detail, username, created_at) VALUES (?, 'add', '품질 검사 통과 (자동 입고)', 'System', CURRENT_TIMESTAMP)",
                (product,)
            )

        conn.commit()
        conn.close()
        self.get_logger().info(f"Logged result '{result}' to database. Inventory updated if PASS.")

    def get_robot_pose_matrix(self, x, y, z, qx, qy, qz, qw):
        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, yaw_cam, robot_pos=None):
        """Converts 3D coordinates and yaw angle from camera to base coordinates using TF."""
        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_color_optical_frame'
            point_cam.point.x = camera_coords[0] / 1000.0
            point_cam.point.y = camera_coords[1] / 1000.0
            point_cam.point.z = camera_coords[2] / 1000.0
            
            # Lookup transform from base_link to camera frame to get rotation
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Transform point to base frame
            point_base = self.tf_buffer.transform(point_cam, 'base_link')
            p_base = [point_base.point.x * 1000.0, point_base.point.y * 1000.0, point_base.point.z * 1000.0]
            
            # Extract rotation from TF to rotate the yaw vector
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            R_cam2base = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            
            # Construct direction vector of block axis in camera frame
            v_cam = np.array([np.cos(yaw_cam), np.sin(yaw_cam), 0.0])
            # Rotate direction vector to base frame
            v_base = R_cam2base @ v_cam
            # Project on table XY plane to compute base yaw
            yaw_base = np.arctan2(v_base[1], v_base[0])
            
            return p_base, yaw_base
            
        except Exception as e:
            self.get_logger().error(f"TF transform_to_base failed: {e}. Falling back to default calculations.")
            return [367.0, 3.0, 81.0], 0.0

    def get_spherical_pose(self, yaw_rad, pitch_deg, distance, target):
        """Computes camera target pose using spherical coordinate system."""
        d = distance
        pitch = np.radians(pitch_deg)
        r_xy = d * np.sin(pitch)
        z = target[2] + d * np.cos(pitch)
        
        c = np.array([
            target[0] + r_xy * np.cos(yaw_rad),
            target[1] + r_xy * np.sin(yaw_rad),
            z
        ])
        t = np.array(target)
        
        z_c = t - c
        norm_z = np.linalg.norm(z_c)
        z_w = np.array([0, 0, 1])
        
        if norm_z < 1e-6:
            z_c = np.array([0, 0, -1])
            y_c = np.array([0, 1, 0])
        else:
            z_c = z_c / norm_z
            y_c = np.cross(z_w, z_c)
            if np.linalg.norm(y_c) < 1e-6:
                arm_vec = np.array([t[0], t[1], 0])
                if np.linalg.norm(arm_vec) < 1e-6:
                    y_c = np.array([0, 1, 0])
                else:
                    arm_dir = arm_vec / np.linalg.norm(arm_vec)
                    y_c = np.cross(z_w, arm_dir)
            else:
                y_c = y_c / np.linalg.norm(y_c)
                
        x_c = np.cross(y_c, z_c)
        R = np.column_stack((x_c, y_c, z_c))
        q = Rotation.from_matrix(R).as_quat()
        
        return c.tolist() + q.tolist()

    def get_tool_pose(self, cam_pose):
        """Converts desired camera pose [x,y,z,qx,qy,qz,qw] in base_link to tool0 pose using TF lookup."""
        T_b2c = np.eye(4)
        T_b2c[:3, 3] = cam_pose[:3]
        T_b2c[:3, :3] = Rotation.from_quat(cam_pose[3:]).as_matrix()
        
        try:
            # Lookup static transform from tool0 to camera_color_optical_frame
            # We use 0.0 seconds for lookup because it is a static transform
            trans = self.tf_buffer.lookup_transform('tool0', 'camera_color_optical_frame', rclpy.time.Time())
            t = trans.transform.translation
            r = trans.transform.rotation
            
            T_t2c = np.eye(4)
            T_t2c[:3, 3] = [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0]
            T_t2c[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            
            # T_base_to_tool0 = T_base_to_cam @ inv(T_tool0_to_cam)
            T_b2t = T_b2c @ np.linalg.inv(T_t2c)
            
            pos = T_b2t[:3, 3].tolist()
            q = Rotation.from_matrix(T_b2t[:3, :3]).as_quat().tolist()
            return pos + q
        except Exception as e:
            self.get_logger().error(f"Failed to transform camera pose to tool pose: {e}. Using camera pose directly.")
            return cam_pose

    def spawn_jenga_mesh(self, x, y, z, yaw):
        """Spawns standard STL mesh in RViz planning scene at computed coordinates."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'jenga_assembly'

        # Load Jenga STL mesh from m0609_rg2_bringup package
        mesh_dir = get_package_share_directory('m0609_rg2_bringup')
        mesh_path = os.path.join(mesh_dir, 'meshes', '이름없음-몸통.stl')

        if os.path.exists(mesh_path):
            try:
                tm = trimesh.load(mesh_path, force='mesh')
                mm_to_m = 0.001
                mesh = Mesh()
                for v in tm.vertices:
                    mesh.vertices.append(Point(x=float(v[0]) * mm_to_m, y=float(v[1]) * mm_to_m, z=float(v[2]) * mm_to_m))
                for f in tm.faces:
                    mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))
                obj.meshes = [mesh]
                
                pose = Pose()
                pose.position.x = x / 1000.0
                pose.position.y = y / 1000.0
                pose.position.z = z / 1000.0
                
                q = Rotation.from_euler('z', yaw).as_quat()
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                
                obj.mesh_poses = [pose]
                self.get_logger().info(f"Loaded mesh from {mesh_path} successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load mesh: {e}. Falling back to default Box.")
                self._apply_fallback_box(obj, x, y, z, yaw)
        else:
            self.get_logger().warn(f"Mesh file not found at {mesh_path}. Falling back to default Box.")
            self._apply_fallback_box(obj, x, y, z, yaw)

        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        self.scene_pub.publish(scene)
        self.get_logger().info("Planning scene update published.")

    def save_home_top_image(self):
        """홈(JReady)에서 top을 찍을 때 YOLO 검출 결과를 그려 사진으로 남긴다."""
        if self.latest_image is None:
            self.get_logger().warn("홈 top 사진 저장 실패 - 카메라 프레임이 아직 없습니다.")
            return
        # YOLO 검출 결과를 프레임에 그려서 저장 (검출 없으면 원본 프레임)
        results = self.model(self.latest_image.copy(), conf=0.60, verbose=False)
        frame_to_save = results[0].plot() if len(results) > 0 else self.latest_image
        possible_paths = [
            os.path.expanduser("~/ws_cobot2_pjt/backend"),
            os.path.expanduser("~/cobot_ws/src/ws_cobot2_pjt/backend"),
        ]
        db_dir = possible_paths[0]
        for p in possible_paths:
            if os.path.exists(p):
                db_dir = p
                break
        save_dir = os.path.join(db_dir, "static", "inspection_images")
        os.makedirs(save_dir, exist_ok=True)
        timestamp = int(time.time() * 1000)
        filename = os.path.join(save_dir, f"inspection_home_top_{timestamp}.jpg")
        if cv2.imwrite(filename, frame_to_save):
            self.get_logger().info(f"홈 top 사진 저장 완료: {filename}")
        else:
            self.get_logger().error(f"홈 top 사진 저장 실패: {filename}")

    def remove_jenga_mesh(self):
        """Removes the spawned Jenga mesh from the MoveIt planning scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'jenga_assembly'
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
        self.scene_pub.publish(scene)
        self.get_logger().info("Removed Jenga mesh from planning scene.")

    def _apply_fallback_box(self, obj, x, y, z, yaw):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            JENGA_WIDTH / 1000.0,
            JENGA_DEPTH / 1000.0,
            JENGA_HEIGHT / 1000.0
        ]  # 75mm x 25mm x 15mm
        
        pose = Pose()
        pose.position.x = x / 1000.0
        pose.position.y = y / 1000.0
        pose.position.z = z / 1000.0
        
        q = Rotation.from_euler('z', yaw).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        obj.primitives = [box]
        obj.primitive_poses = [pose]

    def calculate_tcp_pose(self, camera_pos, target_pos, y_offset=0.0):
        """Spherical projection calculator (same as auto_dataset_capture_node)."""
        c = np.array(camera_pos)
        t = np.array(target_pos)
        z_c = t - c
        norm_z = np.linalg.norm(z_c)
        z_w = np.array([0, 0, 1])
        
        if norm_z < 1e-6:
            z_c = np.array([0, 0, -1])
            y_c = np.array([0, 1, 0])
        else:
            z_c = z_c / norm_z
            y_c = np.cross(z_w, z_c)
            if np.linalg.norm(y_c) < 1e-6:
                arm_vec = np.array([t[0], t[1], 0])
                if np.linalg.norm(arm_vec) < 1e-6:
                    y_c = np.array([0, 1, 0])
                else:
                    arm_dir = arm_vec / np.linalg.norm(arm_vec)
                    y_c = np.cross(z_w, arm_dir)
            else:
                y_c = y_c / np.linalg.norm(y_c)
                
        x_c = np.cross(y_c, z_c)
        R = np.column_stack((x_c, y_c, z_c))
        
        tcp_pos = c - y_offset * y_c
        q = Rotation.from_matrix(R).as_quat() # [x, y, z, w]
        return tcp_pos.tolist() + q.tolist()

    def handle_run_jenga_inspection(self, request, response):
        """Service callback that spawns the control loop in a separate thread to prevent deadlocks."""
        if self.hmi_stop_requested:
            response.success = False
            response.message = "HMI 긴급정지 상태라 검사를 시작할 수 없습니다."
            return response
        self.get_logger().info("Starting Jenga Inspection sequence in a separate thread...")
        thread = threading.Thread(target=self._run_inspection_thread, args=(request, response))
        thread.start()
        
        while rclpy.ok() and thread.is_alive():
            time.sleep(0.1)
            
        return response

    def _run_inspection_thread(self, request, response):
        """Background thread executing the Jenga inspection and scanning sequence."""
        JReady = [-62.57, 2.16, 76.81, 0.00, 100.99, -62.44]

        # 컨베이어로 검사장소까지 이동시키고, 도착할 때까지 기다린 뒤에 검사를 시작한다.
        if self.conveyor is not None:
            self.get_logger().info(
                f"컨베이어를 검사장소로 이동합니다 (MOVE:{CONVEYOR_MOVE_TO_INSPECTION_STEPS})..."
            )
            self.conveyor.move(CONVEYOR_MOVE_TO_INSPECTION_STEPS)
            deadline = time.time() + CONVEYOR_MOVE_TO_INSPECTION_WAIT_SEC
            while time.time() < deadline:
                if self.hmi_stop_requested:
                    response.success = False
                    response.message = "HMI 긴급정지로 검사 시작 전 중단됨"
                    return
                time.sleep(0.1)
            self.get_logger().info("컨베이어 도착 예상 - 검사를 시작합니다.")
        else:
            self.get_logger().warn("컨베이어 미연결 - 이동 없이 바로 검사를 시작합니다.")

        # Initialize/Reset states at start of inspection
        self.hand_detected = False
        self.current_hand_pos = None
        self.in_evasion = False
        self.clear_hand_obstacle()

        # 젠가 검사 시작 전 그리퍼 열기 명령
        try:
            self.get_logger().info("Opening OnRobot RG2 gripper at start of inspection...")
            self.gripper.open_gripper()
            time.sleep(1.0) # 그리퍼가 완전히 열릴 때까지 잠시 대기
        except Exception as e:
            self.get_logger().error(f"Failed to open gripper: {e}")
        
        # Helper to ensure we move to JReady initially, with hand evasion check
        def move_to_initial_pose():
            while rclpy.ok():
                self.get_logger().info("Moving to initial scan pose JReady...")
                if self.move_to_joints_moveit(JReady):
                    return True
                if self.hand_detected:
                    self.get_logger().warn("Hand detected during initial pose movement. Evading to JReady...")
                    self.execute_safety_evasion()
                    self.get_logger().info("Waiting for hand to clear before resuming...")
                    while rclpy.ok() and self.hand_detected:
                        time.sleep(0.5)
                else:
                    # Generic motion planning failure (e.g. workspace limitation)
                    return False
            return False

        if not move_to_initial_pose():
            response.success = False
            response.message = "Failed to move to initial scan pose"
            return
                
        time.sleep(1.5) # Settle camera

        # Get real Jenga position via service call (ensure hand is cleared first)
        while rclpy.ok() and self.hand_detected:
            self.get_logger().warn("Hand detected. Waiting for hand to clear before camera scan...")
            time.sleep(1.0)

        self.get_logger().info("Calling /get_jenga_position service from side view...")
        if not self.get_position_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = "Perception node '/get_jenga_position' not available"
            return

        pos_req = SrvDepthPosition.Request()
        pos_req.target = "top"
        future = self.get_position_client.call_async(pos_req)
        
        # Wait for future to complete
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
            
        res = future.result()
        if not res or len(res.depth_position) < 4 or sum(res.depth_position[:3]) == 0:
            response.success = False
            response.message = "Failed to detect Jenga block 'top' or coordinates out of range"
            return
  
        x_cam, y_cam, z_cam, yaw_cam = res.depth_position[:4]
        self.get_logger().info(f"Detected Jenga camera pose: x={x_cam:.2f}, y={y_cam:.2f}, z={z_cam:.2f}, yaw={yaw_cam:.4f}")

        # 홈에서 top을 찍은 시점의 사진을 남긴다
        self.save_home_top_image()
  
        # Translate to Base frame using non-blocking TF lookup
        robot_posx = self.get_current_pose_tf()
        p_base, yaw_base = self.transform_to_base([x_cam, y_cam, z_cam], yaw_cam, robot_posx)
        self.get_logger().info(f"Translated Base pose: x={p_base[0]:.2f}, y={p_base[1]:.2f}, z={p_base[2]:.2f}, yaw={np.degrees(yaw_base):.2f}°")
  
        # 3. Spawn STL mesh in RViz
        spawn_z = p_base[2] - 42.0
        self.last_jenga_spawn_params = (p_base[0], p_base[1], spawn_z, yaw_base)
        self.spawn_jenga_mesh(p_base[0], p_base[1], spawn_z, yaw_base)
  
        # 4. Generate scanning viewpoints around the Jenga block using Spherical coordinates
        p_target = [p_base[0], p_base[1], p_base[2] - 42.0]
        face_names = ["Front", "Right", "Back", "Left"]
        
        # 다중 각도 스윕으로 복구 (다양한 각도에서 교차 검증)
        target_pitches = [37.0, 45.0, 52.0, 60.0]
        face_successful_poses = {i: [] for i in range(4)}
        
        for i in range(4):
            theta = yaw_base + i * (np.pi / 2.0)
            for p in target_pitches:
                joint_angles = None
                for d in [280.0, 250.0, 310.0]:
                    target_pose = self.get_spherical_pose(theta, p, d, p_target)
                    tool_pose = self.get_tool_pose(target_pose)
                    joint_angles = self.solve_ik(tool_pose)
                    if joint_angles:
                        break
                if joint_angles:
                    face_successful_poses[i].append((p, joint_angles))
                    self.get_logger().info(f"IK solution FOUND for {face_names[i]} Face at Pitch {p}°")
                else:
                    self.get_logger().warn(f"IK solution FAILED for {face_names[i]} Face at Pitch {p}°")

        # 6층 맵 패턴 매칭을 위해 면1(Right)과 면2(Back)를 스캔
        # 만약 둘 중 하나라도 IK가 실패하면 인접한 다른 2면을 찾음
        if len(face_successful_poses[1]) > 0 and len(face_successful_poses[2]) > 0:
            best_pair = (1, 2)
        else:
            adjacent_pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
            best_pair = None
            for idx1, idx2 in adjacent_pairs:
                if len(face_successful_poses[idx1]) > 0 and len(face_successful_poses[idx2]) > 0:
                    best_pair = (idx1, idx2)
                    break

        if best_pair is None:
            self.get_logger().error("No valid IK poses found for any adjacent faces. Aborting inspection.")
            response.success = False
            response.message = "Failed to find any valid IK configurations"
            return
            
        idx1, idx2 = best_pair
        self.get_logger().info(f"Selected Adjacent Faces for 6-Floor Map: {face_names[idx1]} and {face_names[idx2]}")

        scan_targets = []
        for p, joints in face_successful_poses[idx1]:
            scan_targets.append((idx1, face_names[idx1], p, joints))
        for p, joints in face_successful_poses[idx2]:
            scan_targets.append((idx2, face_names[idx2], p, joints))

        self.inspection_data = {}
        self.current_inspection_images = []
        face_results_buffer = {idx1: [], idx2: []}

        # Iterate targets with safety loop
        target_idx = 0
        scene_change_retry_counts = {}
        while target_idx < len(scan_targets):
            face_idx, face_name, pitch_deg, joint_angles = scan_targets[target_idx]
            self.current_pitch_deg = pitch_deg
            self.get_logger().info(f"Moving to Scan Position: {face_name} Face, Pitch {pitch_deg}°...")
            
            # Try to move to the scan position
            success = self.move_to_joints_moveit(joint_angles)
            if not success:
                if self.hand_detected:
                    self.get_logger().warn(f"Hand detected while moving to {face_name} Face! Initiating safety evasion...")
                    self.execute_safety_evasion()
                    
                    self.get_logger().warn("Waiting for hand to clear before resuming scan...")
                    while rclpy.ok() and self.hand_detected:
                        time.sleep(0.5)
                        
                    self.get_logger().info(f"Hand cleared. Resuming scan of {face_name} Face (retrying same position)...")
                    continue
                elif self.last_moveit_error_code == -3:
                    scene_change_retry_counts[target_idx] = scene_change_retry_counts.get(target_idx, 0) + 1
                    if scene_change_retry_counts[target_idx] > 3:
                        self.get_logger().error(
                            f"MoveIt scene changed repeatedly while moving to {face_name} Face. "
                            "Skipping this scan position after 3 retries."
                        )
                        target_idx += 1
                        continue

                    self.get_logger().warn(
                        f"MoveIt scene changed while moving to {face_name} Face. "
                        f"Waiting briefly and retrying the same scan position "
                        f"({scene_change_retry_counts[target_idx]}/3)..."
                    )
                    time.sleep(1.0)
                    continue
                else:
                    self.get_logger().error(f"Failed to move to {face_name} Face due to non-safety reasons. Skipping.")
                    target_idx += 1
                    continue

            self.get_logger().info("카메라 앵글 흔들림 보정을 위해 0.5초 대기 후 자동 촬영을 시작합니다...")
            time.sleep(0.5) # Settle camera

            # Double check if hand was detected during settling time
            if self.hand_detected:
                self.get_logger().warn(f"Hand detected right after arrival at {face_name} Face! Evading...")
                self.execute_safety_evasion()
                self.get_logger().warn("Waiting for hand to clear before resuming scan...")
                while rclpy.ok() and self.hand_detected:
                    time.sleep(0.5)
                self.get_logger().info("Hand cleared. Resuming scan...")
                continue

            # 단일 프레임 직접 캡처 및 깊이 역산
            holes = self.capture_single_frame(face_name, pitch_deg)
            face_results_buffer[face_idx].extend(holes)

            # Move on to next target
            target_idx += 1

        # 모든 캡처 종료 후 다중 각도 필터링 수행
        from collections import Counter
        pos_mirror = {"Left": "Right", "Center": "Center", "Right": "Left"}
        for f_idx in face_results_buffer:
            face_name_str = face_names[f_idx]
            counter = Counter(face_results_buffer[f_idx])
            
            # 4번의 각도 중 최소 1개 각도에서만 보이면 실제 구멍으로 인정 (사용자 요청에 따라 조건 완화)
            final_holes = [hole for hole, count in counter.items() if count >= 1]
            self.get_logger().info(f"[{face_name_str}] 면 다중 각도 종합 필터링된 최종 구멍: {final_holes}")
            
            # Left, Front 면이 스캔되었을 경우 6층 맵 매칭을 위해 Right, Back 면 시점으로 좌우 반전(Mirror) 정규화
            if f_idx == 1:
                self.inspection_data[1] = final_holes
            elif f_idx == 3: # Left
                self.inspection_data[1] = [(f, pos_mirror.get(p, p)) for f, p in final_holes]
            elif f_idx == 2:
                self.inspection_data[2] = final_holes
            elif f_idx == 0: # Front
                self.inspection_data[2] = [(f, pos_mirror.get(p, p)) for f, p in final_holes]

        # 5. Analyze defects (Build 6-floor map and match patterns)
        failed_reasons = []
        is_pass = self.analyze_defects()
        final_result = "PASS" if is_pass else "FAIL"
        if not is_pass:
            failed_reasons.append("Jenga 6-Floor assembly pattern does not match reference templates (Detected wrong hole patterns)")
        import json
        map_data_json = json.dumps(self.final_jenga_map) if hasattr(self, 'final_jenga_map') else None
        
        defect_loc = None
        if not is_pass:
            if hasattr(self, 'final_jenga_map'):
                missing_locations = []
                for floor in range(1, 7):
                    if floor not in self.final_jenga_map:
                        continue
                    blocks = self.final_jenga_map[floor]
                    for idx, block in enumerate(blocks):
                        if block == "X":
                            if floor % 2 != 0:
                                # 홀수층: 앞면(Front) 기준 (현재 DB맵은 뒷면 기준이므로 0과 2 좌우 반전)
                                pos_str = "우측" if idx == 0 else "좌측" if idx == 2 else "중앙"
                            else:
                                # 짝수층: 오른쪽(Right) 기준 (DB맵과 시점 일치)
                                pos_str = "좌측" if idx == 0 else "우측" if idx == 2 else "중앙"
                            missing_locations.append(f"{floor}층 {pos_str}")
                
                if missing_locations:
                    defect_loc = ", ".join(missing_locations) + " 누락"
                else:
                    defect_loc = failed_reasons[0]
            else:
                defect_loc = failed_reasons[0]

        # 백엔드 모델에 맞춰 DB에 저장 (정상 제품일 경우 몇형 제품인지 저장, 아닐 경우 불량품 표기)
        product_name_for_db = self.final_matched_product if is_pass else "알 수 없는 패턴 (불량품)"
        self.log_result_to_db(
            product_name_for_db,
            final_result,
            defect_loc,
            map_data_json,
            image_paths=self.current_inspection_images,
        )

        # 합격이면 컨베이어를 마저 보낸다 - 이후로는 로봇팔이 할 일이 없어서
        # (검사 실패 시엔 안 보내고, 완료를 기다리지도 않음) 그냥 보내기만 함.
        if is_pass and self.conveyor is not None:
            self.get_logger().info(
                f"검사 합격 - 컨베이어를 마저 이동합니다 (MOVE:{CONVEYOR_MOVE_TO_PASS_STEPS})"
            )
            self.conveyor.move(CONVEYOR_MOVE_TO_PASS_STEPS)

        # 불합격이면 바로 젠가 블록을 밀어낸다 (접근 자세 → 미는 자세)
        if not is_pass:
            self.get_logger().info("검사 불합격 - 젠가 블록을 밀어냅니다...")
            def check_hand_before_action(name):
                if self.hand_detected or self.push_hand_seen:
                    self.get_logger().error(f"손 감지됨 - {name} 전 안전을 위해 밀기 동작을 전면 취소(Abort)합니다.")
                    return False
                return True

            def push_move_check_hand(target_joints, name):
                scene_change_retry_count = 0
                while rclpy.ok():
                    if not check_hand_before_action(name):
                        return "abort"
                    
                    # 이동 직전에 Octomap을 비워서, 미리 손을 피하는 우회 경로를 생성하지 못하게 함
                    self.call_clear_octomap()
                    time.sleep(0.1)  # Clear 반영 대기
                    
                    if self.move_to_joints_moveit(target_joints):
                        return "success"
                    
                    if self.hand_detected or self.last_moveit_error_code == 'HAND_DETECTED':
                        self.get_logger().error(f"{name} 중 손 감지됨 - 안전을 위해 밀기 동작을 전면 취소(Abort)합니다.")
                        return "abort"
                    elif self.last_moveit_error_code == -3:
                        # 궤적 생성 시점엔 Octomap이 비어있어 직선으로 출발했지만, 이동 중 손(장애물)이 나타난 경우
                        self.get_logger().error(f"{name} 중 손에 의해 경로 막힘(MoveIt -3) - 밀기를 즉시 취소(Abort)합니다.")
                        return "abort"
                    else:
                        self.get_logger().error(f"{name} 이동 실패 - 밀기 동작 건너뜀.")
                        return "failed"
                return "failed"

            max_push_attempts = 5
            for attempt in range(max_push_attempts):
                self.in_push_sequence = True
                self.push_hand_seen = False
                self.push_cancel_requested = False
                self.clear_hand_obstacle()
                time.sleep(0.3)
                push_aborted = False

                while rclpy.ok():
                    approach_result = push_move_check_hand(PUSH_APPROACH_JOINTS_DEG, "밀기 접근 자세")
                    if approach_result == "abort":
                        self.get_logger().error("밀기 접근 중 손이 감지되어 밀기 시퀀스를 완전히 중단합니다.")
                        push_aborted = True
                        break
                    if approach_result != "success":
                        break

                    if not check_hand_before_action("그리퍼 닫기"):
                        self.get_logger().error("그리퍼 닫기 전 손이 감지되어 밀기 시퀀스를 완전히 중단합니다.")
                        push_aborted = True
                        break

                    # 접근 자세까지는 젠가 메시를 장애물로 유지하고, 실제 밀기 동작 전 제거한다.
                    self.remove_jenga_mesh()
                    time.sleep(0.5)  # 씬 업데이트 반영 대기
                    try:
                        self.get_logger().info("밀기 전 OnRobot RG2 그리퍼를 닫습니다...")
                        self.gripper.close_gripper()
                        time.sleep(1.0)
                    except Exception as e:
                        self.get_logger().error(f"밀기 전 그리퍼 닫기 실패: {e}")

                    target_result = push_move_check_hand(PUSH_TARGET_JOINTS_DEG, "밀기 목표 자세")
                    if target_result == "success":
                        break
                    if target_result == "abort":
                        self.get_logger().error(
                            "밀기 목표 자세 이동 중 손 감지됨 - 미는 모션을 즉시 취소하고 그리퍼를 엽니다."
                        )
                        try:
                            self.gripper.open_gripper()
                            time.sleep(1.0)
                        except Exception as e:
                            self.get_logger().error(f"복귀 전 그리퍼 열기 실패: {e}")
                        
                        if getattr(self, 'last_jenga_spawn_params', None) is not None:
                            self.spawn_jenga_mesh(*self.last_jenga_spawn_params)
                            time.sleep(0.5)
                            
                        push_aborted = True
                        break
                    break

                self.in_push_sequence = False
                self.push_hand_seen = False
                self.push_cancel_requested = False
                self.clear_hand_obstacle()

                if push_aborted:
                    if attempt < max_push_attempts - 1:
                        recovered = self.recover_to_push_approach()
                        if recovered:
                            self.get_logger().warn("밀기 1 위치 복귀 완료. 2초간 안전 대기 후 다시 밀기를 시도합니다.")
                        else:
                            self.get_logger().error("밀기 1 위치 복귀 실패. 안전을 위해 밀기 재시도를 중단합니다.")
                            break
                        time.sleep(2.0)

                        self.get_logger().warn(f"안전 확보됨. 다시 밀기를 시도합니다... (남은 재시도: {max_push_attempts - attempt - 1})")
                    else:
                        self.get_logger().error("밀기 재시도 횟수를 초과하여 불량품 배출을 포기합니다.")
                else:
                    # 밀기 성공 혹은 손 감지가 아닌 다른 이유로 실패한 경우 탈출
                    break

        # 6. Return back to Home position with safety evasion checks
        self.get_logger().info("Inspection complete. Returning to Home...")
        home_retries = 0
        MAX_HOME_RETRIES = 5
        while rclpy.ok():
            if self.move_to_joints_moveit(JReady):
                break
            if self.hand_detected:
                self.execute_safety_evasion()
                while rclpy.ok() and self.hand_detected:
                    time.sleep(0.5)
            else:
                # 손 감지가 아닌 사유(계획 실패 등)로 홈 복귀에 실패해도 바로 포기하지 않고
                # 재시도한다. 도달 불가한 자세에서 무한 대기하는 것을 막기 위해 횟수는 제한한다.
                home_retries += 1
                if home_retries >= MAX_HOME_RETRIES:
                    self.get_logger().error(
                        f"홈 복귀 {MAX_HOME_RETRIES}회 실패 - 홈 복귀를 포기합니다.")
                    break
                self.get_logger().warn(
                    f"홈 복귀 실패(손 감지 외 사유) - 재시도 {home_retries}/{MAX_HOME_RETRIES}...")
                time.sleep(1.0)

        # Cleanup obstacles
        self.clear_hand_obstacle()

        # Format output message
        if final_result == "PASS":
            response.success = True
            response.message = f"Jenga Inspection: PASS ({getattr(self, 'final_matched_product', 'Matched')})"
        else:
            response.success = False
            response.message = f"Jenga Inspection: FAIL (Unknown Pattern)"
            
        return response


    def destroy_node(self):
        """Clean up the helper node and executor upon destruction."""
        if self.conveyor is not None:
            self.conveyor.close()
        self.action_executor.shutdown()
        self.action_node.destroy_node()
        super().destroy_node()


def main(args=None):
    node = JengaInspectorNode()
    
    # Use MultiThreadedExecutor so that blocking service callbacks do not deadlock action client callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    # Keep node alive in main thread
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == "__main__":
    main()
