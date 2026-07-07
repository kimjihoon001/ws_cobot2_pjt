#!/usr/bin/env python3
import sys
import os
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print("ultralytics 라이브러리가 필요합니다. 'pip install ultralytics' 를 실행해주세요.")
    sys.exit(1)

import DR_init
# 두산 로봇 모델 정보 (환경에 맞게 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('defect_inspector_node')
        
        # YOLOv8 ONNX 모델 로드
        model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best_2.onnx'
        if not os.path.exists(model_path):
            model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best_2.onnx' # fallback
            
        self.get_logger().info(f'Loading YOLOv8 ONNX model from: {model_path}')
        self.model = YOLO(model_path, task='detect')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.depth_frame = None
        self.intrinsics = None
        
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.image_pub = self.create_publisher(Image, '/yolo/result_image', 10)
        
        self.OBJECT_POS = [367.0, 3.0, 30.0]
        self.CAMERA_Y_OFFSET = 75.0
        self.VELOCITY = 60
        self.ACC = 60
        
        self.current_pitch_deg = 45.0
        self.FIXED_DISTANCE = 280.0
        
        self.robot_lock = threading.Lock()
        
        # 검사 데이터
        self.inspection_data = {}
        
        self.get_logger().info('=========================================')
        self.get_logger().info(' 자동 불량품 검출(Inspection) 노드 시작')
        self.get_logger().info(' (45도 피치, 280mm 반경 기준)')
        self.get_logger().info('=========================================')
        
        # 백그라운드 스레드에서 자동 검사 시퀀스 즉시 시작
        threading.Thread(target=self.run_inspection_sequence).start()

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            pass

    def get_vertical_depth(self, u, v, Z_c, pitch_deg=45.0):
        if Z_c <= 0 or self.intrinsics is None:
            return None
            
        theta = np.radians(90.0 - pitch_deg)
        fx = self.intrinsics['fx']
        ppx = self.intrinsics['ppx']
        
        X_c = (u - ppx) * Z_c / fx
        virtual_Y_c = -X_c
        
        vertical_depth = Z_c * np.sin(theta) + virtual_Y_c * np.cos(theta)
        return vertical_depth

    def calculate_tcp_pose(self, camera_pos, target_pos, y_offset=0.0):
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
        
        beta = np.arctan2(np.sqrt(R[2,0]**2 + R[2,1]**2), R[2,2])
        if np.abs(beta) < 1e-6:
            alpha = 0.0
            gamma = np.arctan2(-R[0,1], R[0,0])
        else:
            alpha = np.arctan2(R[1,2], R[0,2])
            gamma = np.arctan2(R[2,1], -R[2,0])
            
        tcp_pos = c - y_offset * y_c
        return tcp_pos.tolist() + np.degrees([alpha, beta, gamma]).tolist()

    def get_spherical_pose(self, angle_deg, pitch_deg, distance):
        target = self.OBJECT_POS[:3]
        d = distance
        pitch = np.radians(pitch_deg)
        rad = np.radians(angle_deg)
        r_xy = d * np.sin(pitch)
        z = target[2] + d * np.cos(pitch)
        
        cam_pos = [target[0] + r_xy * np.cos(rad), target[1] + r_xy * np.sin(rad), z]
        return self.calculate_tcp_pose(cam_pos, target, self.CAMERA_Y_OFFSET)

    def move_to_yaw(self, target_yaw, current_yaw):
        from DSR_ROBOT2 import movel, movec, mwait
        
        self.get_logger().info(f"==> 로봇이 검사 위치(Yaw: {target_yaw}도)로 이동합니다...")
        pos_target = self.get_spherical_pose(target_yaw, self.current_pitch_deg, self.FIXED_DISTANCE)
        
        try:
            if current_yaw is None or current_yaw == target_yaw:
                with self.robot_lock:
                    movel(pos_target, vel=self.VELOCITY, acc=self.ACC)
            else:
                via_yaw = (current_yaw + target_yaw) / 2.0
                pos_via = self.get_spherical_pose(via_yaw, self.current_pitch_deg, self.FIXED_DISTANCE)
                with self.robot_lock:
                    movec(pos_via, pos_target, vel=self.VELOCITY, acc=self.ACC)
                
            mwait()
            self.get_logger().info(f"==> 이동 완료.")
        except Exception as e:
            self.get_logger().error(f"이동 중 오류 발생: {e}")

    def capture_and_process_frames(self, yaw_face):
        frames_collected = 0
        face_data = []
        
        while frames_collected < 5:
            if self.latest_image is None or self.depth_frame is None or self.intrinsics is None:
                time.sleep(0.1)
                continue
                
            img_copy = self.latest_image.copy()
            # confidence threshold를 0.20으로 낮춰 탐지율을 높임
            results = self.model(img_copy, conf=0.20, verbose=False)
            annotated_frame = results[0].plot()
            
            try:
                entire_box = None
                hole_boxes = []
                names = results[0].names
                for box in results[0].boxes:
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
                        entire_v_depth = self.get_vertical_depth(valid_eu, valid_ev, valid_entire_Z, pitch_deg=self.current_pitch_deg)
                        if entire_v_depth is not None:
                            theta = np.radians(90.0 - self.current_pitch_deg)
                            fx = self.intrinsics['fx']
                            ppx = self.intrinsics['ppx']
                            
                            X_ent_c = (valid_eu - ppx) * valid_entire_Z / fx
                            K = -np.sin(theta) * X_ent_c - np.cos(theta) * valid_entire_Z
                            
                            denom_base = -np.sin(theta) * (ex1 - ppx) / fx - np.cos(theta)
                            if abs(denom_base) > 1e-6:
                                Z_base_true = K / denom_base
                                base_v_depth = self.get_vertical_depth(ex1, ev, Z_base_true, pitch_deg=self.current_pitch_deg)
                            else:
                                base_v_depth = entire_v_depth
                            
                            for box in hole_boxes:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                u = int((x1 + x2) / 2)
                                v = int((y1 + y2) / 2)
                                
                                denom = -np.sin(theta) * (u - ppx) / fx - np.cos(theta)
                                if abs(denom) > 1e-6:
                                    Z_hole_true = K / denom
                                    obj_v_depth = self.get_vertical_depth(u, v, Z_hole_true, pitch_deg=self.current_pitch_deg)
                                    if obj_v_depth is not None:
                                        height_from_base = base_v_depth - obj_v_depth
                                        compensated_height = height_from_base - 7.0
                                        floor_num = max(1, int(round(compensated_height / 14.0)) + 1)
                                        
                                        tower_width = ey2 - ey1
                                        third = tower_width / 3.0
                                        if v < ey1 + third:
                                            horiz_pos = "Left"
                                        elif v < ey1 + 2 * third:
                                            horiz_pos = "Center"
                                        else:
                                            horiz_pos = "Right"
                                            
                                        face_data.append({"floor": floor_num, "pos": horiz_pos})
                                        
                                        cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                                        cv2.putText(annotated_frame, f"{horiz_pos} {floor_num}F", (u+10, v-10),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    frames_collected += 1
                    self.get_logger().info(f"[{yaw_face}도] 검사 프레임 수집 및 판별 중... ({frames_collected}/5)")
            except Exception as e:
                self.get_logger().error(f"오류: {e}")
                
            try:
                # 처리된 결과를 rqt_image_view 등으로 볼 수 있도록 토픽 발행
                msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(msg)
            except:
                pass
            
            time.sleep(0.2) # 프레임 간격
            
        self.inspection_data[yaw_face] = face_data

    def analyze_defects(self):
        self.get_logger().info("\n=========================================")
        self.get_logger().info("=== 젠가 타워(6층) 통합 상태 맵 ===")
        self.get_logger().info("범례: [O 정상(블록)] [X 불량(누락/구멍)]")
        self.get_logger().info("-" * 55)
        
        # 1층부터 6층까지 각 면의 맵 초기화
        jenga_map = {
            f: {
                -45: {"Left": "O", "Center": "O", "Right": "O"},
                45:  {"Left": "O", "Center": "O", "Right": "O"}
            } for f in range(1, 7)
        }
        
        defect_count = 0
        for yaw, holes in self.inspection_data.items():
            counts = {}
            for h in holes:
                if h["floor"] <= 6:
                    key = (h["floor"], h["pos"])
                    counts[key] = counts.get(key, 0) + 1
            
            valid_holes = [k for k, v in counts.items() if v >= 3]
            
            for floor, pos in valid_holes:
                if floor <= 6:
                    jenga_map[floor][yaw][pos] = "X"
                    defect_count += 1
        
        # 젠가 특성을 반영한 통합 맵 구축 (홀수층=면1, 짝수층=면2)
        unified_pattern = []
        for f in range(6, 0, -1):
            if f % 2 != 0:
                # 홀수층
                face_data = jenga_map[f][-45]
                view_str = "면1(-45도) 관측"
            else:
                # 짝수층
                face_data = jenga_map[f][45]
                view_str = "면2( 45도) 관측"
                
            map_str = f"{face_data['Left']} {face_data['Center']} {face_data['Right']}"
            pattern_str = map_str.replace(" ", "")
            unified_pattern.append(pattern_str)
            
            self.get_logger().info(f"[{f}층] {map_str}  ({view_str})")
            
        self.get_logger().info("-" * 55)
        
        # 1층 -> 6층 순서로 튜플화
        unified_pattern.reverse()
        current_map_tuple = tuple(unified_pattern)
        
        # 견본 양품 템플릿 정의 (단일 타워 1층 -> 6층)
        REFERENCE_TEMPLATES = {
            "기본 완제품 (누락 없음)": (
                "OOO", "OOO", "OOO", "OOO", "OOO", "OOO"
            ),
            "A형 상품 (3, 5층 중앙 누락)": (
                "OOO", "OOO", "OXO", "OOO", "OOO", "OOO"
            ),
            "B형 상품 (짝수층 양끝 누락)": (
                "OOO", "OOO", "OOO", "XOX", "OOO", "OOO"
            ),
            "C형 상품 (2층 중앙, 4층 중앙 누락)": (
                "OOO", "OXO", "OOO", "OXO", "OOO", "OOO"
            )
        }
        
        # 패턴 매칭 및 상품 판정
        matched_product = None
        for product_name, template in REFERENCE_TEMPLATES.items():
            if current_map_tuple == template:
                matched_product = product_name
                break
                
        if matched_product:
            self.get_logger().info(f"✅ 판정 결과: 일치하는 견본을 찾았습니다! => [{matched_product}]")
        else:
            self.get_logger().warn(f"❌ 판정 결과: 일치하는 견본이 없습니다! => [미등록 불량품]")
            self.get_logger().warn(f"   (발견된 총 구멍 수: {defect_count}개)")
            
        self.get_logger().info("=========================================\n")

    def run_inspection_sequence(self):
        yaws_to_inspect = [-45, 45] # 인접한 두 면
        current_yaw = None
        
        for yaw in yaws_to_inspect:
            self.move_to_yaw(yaw, current_yaw)
            current_yaw = yaw
            
            # 사용자 신호 대기 (입력 대기)
            self.get_logger().info(f"\n>> [{yaw}도] 로봇 이동 완료! <<")
            input(f"카메라가 안정되면 터미널 창에서 [엔터(Enter)] 키를 눌러 검사(촬영)를 시작하세요...\n")
            
            self.get_logger().info("검사(프레임 수집) 시작...")
            self.capture_and_process_frames(yaw)
            
        self.analyze_defects()
        
        self.get_logger().info("모든 검사가 완료되었습니다. 프로세스를 유지합니다 (rqt 등으로 화면 확인 가능).")
        self.get_logger().info("종료하려면 터미널에서 Ctrl+C 를 누르세요.")

def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_yolo_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    node = YoloInferenceNode()
    
    try:
        rclpy.spin(node) # 더 이상 실시간 cv2 루프를 돌지 않고 ROS2 콜백만 처리합니다.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
