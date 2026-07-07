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
        super().__init__('yolo_inference_node')
        
        # YOLOv8 ONNX 모델 로드
        model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best_2.onnx'
        if not os.path.exists(model_path):
            model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best_2.onnx' # fallback
            
        self.get_logger().info(f'Loading YOLOv8 ONNX model from: {model_path}')
        self.model = YOLO(model_path, task='detect')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.new_image_available = False
        self.depth_frame = None
        self.intrinsics = None
        
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.image_pub = self.create_publisher(Image, '/yolo/result_image', 10)
        
        self.is_manual_mode = False
        self.is_moving = False
        
        self.OBJECT_POS = [367.0, 3.0, 30.0]
        self.CAMERA_Y_OFFSET = 75.0
        self.VELOCITY = 60
        self.ACC = 60
        self.FIXED_DISTANCE = 300.0
        self.current_pitch_deg = 45.0
        self.current_yaw = None
        
        self.robot_lock = threading.Lock()
        
        self.last_inference_time = 0.0
        self.inference_interval = 0.2 # 초당 5프레임 제한
        
        self.get_logger().info('=========================================')
        self.get_logger().info(' YOLO Inference Node (Relative Depth Mode) Started.')
        self.get_logger().info(' [조작 방법]')
        self.get_logger().info(' M   : 수동 제어(직접 교시) On/Off 토글')
        self.get_logger().info(' 1   : right_back (-135도) 이동')
        self.get_logger().info(' 2   : right_front (-45도) 이동')
        self.get_logger().info(' 3   : left_front (45도) 이동')
        self.get_logger().info(' 4/5/6: 피치(내려다보는 각도)를 30/45/60도로 변경')
        self.get_logger().info(' ESC : 프로그램 종료')
        self.get_logger().info('=========================================')

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.new_image_available = True
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
        
        # 카메라가 시계방향 90도로 누워있으므로, 화면의 가로(u)축이 실제 세상의 세로(높이) 방향입니다.
        # 왼쪽(u=0)이 바닥, 오른쪽(u=width)이 꼭대기를 향하므로, 기존의 Y_c 역할을 -X_c가 대신합니다.
        X_c = (u - ppx) * Z_c / fx
        virtual_Y_c = -X_c
        
        vertical_depth = Z_c * np.sin(theta) + virtual_Y_c * np.cos(theta)
        return vertical_depth

    # ===== 로봇 이동 관련 수식 =====
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

    def move_routine(self, target_yaw, name):
        from DSR_ROBOT2 import movel, movec, mwait
        
        self.is_moving = True
        self.get_logger().info(f"==> {name} (각도: {target_yaw}도, 피치: {self.current_pitch_deg}도) 위치로 이동합니다...")
        
        pos_target = self.get_spherical_pose(target_yaw, self.current_pitch_deg, self.FIXED_DISTANCE)
        
        old_yaw = self.current_yaw
        self.current_yaw = target_yaw
        
        try:
            if old_yaw is None or old_yaw == target_yaw:
                with self.robot_lock:
                    movel(pos_target, vel=self.VELOCITY, acc=self.ACC)
            else:
                via_yaw = (old_yaw + target_yaw) / 2.0
                pos_via = self.get_spherical_pose(via_yaw, self.current_pitch_deg, self.FIXED_DISTANCE)
                with self.robot_lock:
                    movec(pos_via, pos_target, vel=self.VELOCITY, acc=self.ACC)
                
            mwait()
            self.get_logger().info(f"==> {name} 도착 완료.")
        except Exception as e:
            self.get_logger().error(f"이동 중 오류 발생: {e}")
        
        self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    
    dsr_node = rclpy.create_node("dsr_yolo_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    
    try:
        from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        sys.exit(1)

    node = YoloInferenceNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            current_time = time.time()
            if node.latest_image is not None and node.new_image_available and (current_time - node.last_inference_time) >= node.inference_interval:
                node.new_image_available = False
                node.last_inference_time = current_time
                
                results = node.model(node.latest_image, verbose=False)
                annotated_frame = results[0].plot()
                
                if node.is_moving:
                    cv2.putText(annotated_frame, "Robot is moving... Pausing depth calculation.", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                elif node.depth_frame is not None and node.intrinsics is not None:
                    try:
                        entire_box = None
                        hole_boxes = []
                        
                        names = results[0].names
                        detected_classes = []
                        for box in results[0].boxes:
                            cls_name = names[int(box.cls[0])]
                            detected_classes.append(cls_name)
                            if cls_name == 'entire':
                                entire_box = box
                            elif cls_name.lower() == 'smallhole':
                                hole_boxes.append(box)
                        
                        # 2. 'entire' 박스가 존재하면 기준점(중심점) 깊이 계산
                        if entire_box is not None:
                            ex1, ey1, ex2, ey2 = entire_box.xyxy[0].cpu().numpy()
                            eu = int((ex1 + ex2) / 2)
                            ev = int((ey1 + ey2) / 2)
                            
                            # 화면이 90도 돌아가 있으므로, 진짜 앞면을 찾으려면 가로(u)가 아니라 세로(v) 방향으로 스캔해야 합니다.
                            valid_entire_Z = float('inf')
                            valid_eu, valid_ev = eu, ev
                            
                            scan_ey_start = int(ey1) + 10
                            scan_ey_end = int(ey2) - 10
                            
                            for scan_v in range(scan_ey_start, scan_ey_end + 1, 5):
                                tu = np.clip(eu, 0, node.depth_frame.shape[1]-1)
                                tv = np.clip(scan_v, 0, node.depth_frame.shape[0]-1)
                                val = float(node.depth_frame[tv, tu])
                                if 0 < val < valid_entire_Z:
                                    valid_entire_Z = val
                                    valid_eu, valid_ev = tu, tv
                            
                            if valid_entire_Z == float('inf'):
                                entire_v_depth = None
                            else:
                                entire_v_depth = node.get_vertical_depth(valid_eu, valid_ev, valid_entire_Z, pitch_deg=node.current_pitch_deg)
                            
                            if entire_v_depth is None:
                                node.get_logger().warn(f"entire 박스 중심({eu}, {ev})의 깊이값이 유효하지 않습니다. (노이즈)")
                            else:
                                # 기준점(entire 중심) 화면에 파란색 원으로 표시
                                cv2.circle(annotated_frame, (valid_eu, valid_ev), 7, (255, 0, 0), -1)
                                # 수학적 평면 교차(Ray-Plane Intersection)를 위한 타워 전면부 평면 상수(K) 계산
                                # 카메라 프레임에서 가상의 수직 평면 방정식: -sin(theta)*X - cos(theta)*Z = K
                                theta = np.radians(90.0 - node.current_pitch_deg)
                                fx = node.intrinsics['fx']
                                ppx = node.intrinsics['ppx']
                                
                                X_ent_c = (valid_eu - ppx) * valid_entire_Z / fx
                                K = -np.sin(theta) * X_ent_c - np.cos(theta) * valid_entire_Z
                                
                                # 1층 바닥 기준점(ex1)의 수직 깊이 계산
                                denom_base = -np.sin(theta) * (ex1 - ppx) / fx - np.cos(theta)
                                if abs(denom_base) > 1e-6:
                                    Z_base_true = K / denom_base
                                    base_v_depth = node.get_vertical_depth(ex1, ev, Z_base_true, pitch_deg=node.current_pitch_deg)
                                else:
                                    base_v_depth = entire_v_depth
                                
                                # 바닥 기준선(1층)을 시각적으로 노란색 선으로 표시
                                cv2.line(annotated_frame, (int(ex1), int(ey1)), (int(ex1), int(ey2)), (0, 255, 255), 2)
                                
                                # 3. 각 'hole'들의 절대 높이 수학적 추정 및 층수 계산
                                for idx, box in enumerate(hole_boxes):
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    u = int((x1 + x2) / 2)
                                    v = int((y1 + y2) / 2)
                                    
                                    # 구멍의 픽셀(u)을 지나는 카메라 광선(Ray)이 가상의 수직 평면과 만나는 Z 깊이 계산
                                    denom = -np.sin(theta) * (u - ppx) / fx - np.cos(theta)
                                    
                                    if abs(denom) > 1e-6:
                                        # 스캔 없이 수학적으로 도출된 구멍의 완벽한 표면 깊이
                                        Z_hole_true = K / denom
                                        obj_v_depth = node.get_vertical_depth(u, v, Z_hole_true, pitch_deg=node.current_pitch_deg)
                                    else:
                                        obj_v_depth = None
                                        
                                    if obj_v_depth is not None:
                                        # 바닥 기준 수직 깊이에서 hole 중심 수직 깊이를 뺌 (구멍 중심의 높이)
                                        height_from_base = base_v_depth - obj_v_depth
                                        
                                        # 구멍(Bounding Box)이 중심 기준이므로, 블록 바닥면 기준으로 보정하기 위해 절반(7.0mm)을 뺌
                                        compensated_height = height_from_base - 7.0
                                        
                                        # 보정된 바닥면 높이를 기준으로 젠가 1층(14.0mm) 단위의 절대 층수 계산
                                        # (round를 통해 가장 가까운 층으로 매핑)
                                        floor_num = max(1, int(round(compensated_height / 14.0)) + 1)
                                        
                                        # 수평 위치(Left, Center, Right) 3등분 판별
                                        tower_width = ey2 - ey1
                                        third = tower_width / 3.0
                                        
                                        if v < ey1 + third:
                                            horiz_pos = "Left"
                                        elif v < ey1 + 2 * third:
                                            horiz_pos = "Center"
                                        else:
                                            horiz_pos = "Right"
                                        
                                        # 터미널에 로그로 수평 위치, 절대 층수와 높이 출력
                                        node.get_logger().info(f"[Hole {idx+1}] 위치: {horiz_pos} / 층수: {floor_num}층 / 바닥대비 높이: {height_from_base:.1f}mm")
                                        
                                        # 화면에 중심점(빨간색 원)과 측정된 위치, 층수 출력
                                        cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                                        cv2.putText(annotated_frame, f"{horiz_pos} {floor_num}F", (u+10, v-10),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                    else:
                                        node.get_logger().warn(f"Hole {idx+1} 중심점 주변 깊이값이 유효하지 않습니다.")
                    except Exception as e:
                        node.get_logger().error(f"상대 깊이 계산 오류: {e}")
                
                cv2.imshow("YOLOv8 Inference & Control", annotated_frame)
                key = cv2.waitKey(20) & 0xFF
                
                if key == 27:
                    break
                    
                if key == ord('m') or key == ord('M'):
                    if node.is_moving:
                        node.get_logger().warn("로봇 이동 중에는 수동 제어를 켤 수 없습니다.")
                        continue
                        
                    if not node.is_manual_mode:
                        node.get_logger().info("==> 수동 제어 모드 ON")
                        try:
                            with node.robot_lock:
                                task_compliance_ctrl()
                            node.is_manual_mode = True
                            node.current_yaw = None
                        except Exception as e:
                            node.get_logger().error(f"수동 제어 모드 켜기 실패: {e}")
                    else:
                        node.get_logger().info("==> 수동 제어 모드 OFF")
                        try:
                            with node.robot_lock:
                                release_compliance_ctrl()
                            node.is_manual_mode = False
                        except Exception as e:
                            node.get_logger().error(f"수동 제어 모드 끄기 실패: {e}")
                
                if key == ord('1'):
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 이동해주세요.")
                    elif not node.is_moving:
                        node.is_moving = True
                        threading.Thread(target=node.move_routine, args=(-135, 'right_back')).start()
                elif key == ord('2'):
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 이동해주세요.")
                    elif not node.is_moving:
                        node.is_moving = True
                        threading.Thread(target=node.move_routine, args=(-45, 'right_front')).start()
                elif key == ord('3'):
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 이동해주세요.")
                    elif not node.is_moving:
                        node.is_moving = True
                        threading.Thread(target=node.move_routine, args=(45, 'left_front')).start()
                        
                elif key == ord('4') or key in [ord('q'), ord('Q')]:
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 조작해주세요.")
                    elif not node.is_moving:
                        if node.current_yaw is not None:
                            node.current_pitch_deg = 30.0
                            node.is_moving = True
                            node.get_logger().info("피치 각도를 30도로 변경하고 이동합니다.")
                            threading.Thread(target=node.move_routine, args=(node.current_yaw, f'pitch_30_yaw_{node.current_yaw}')).start()
                        else:
                            node.get_logger().warn("현재 방향(Yaw)이 설정되지 않아 피치를 변경할 수 없습니다. 1, 2, 3번 키를 먼저 눌러주세요.")
                            
                elif key == ord('5') or key in [ord('w'), ord('W')]:
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 조작해주세요.")
                    elif not node.is_moving:
                        if node.current_yaw is not None:
                            node.current_pitch_deg = 45.0
                            node.is_moving = True
                            node.get_logger().info("피치 각도를 45도로 변경하고 이동합니다.")
                            threading.Thread(target=node.move_routine, args=(node.current_yaw, f'pitch_45_yaw_{node.current_yaw}')).start()
                        else:
                            node.get_logger().warn("현재 방향(Yaw)이 설정되지 않아 피치를 변경할 수 없습니다. 1, 2, 3번 키를 먼저 눌러주세요.")
                            
                elif key == ord('6') or key in [ord('e'), ord('E')]:
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 조작해주세요.")
                    elif not node.is_moving:
                        if node.current_yaw is not None:
                            node.current_pitch_deg = 60.0
                            node.is_moving = True
                            node.get_logger().info("피치 각도를 60도로 변경하고 이동합니다.")
                            threading.Thread(target=node.move_routine, args=(node.current_yaw, f'pitch_60_yaw_{node.current_yaw}')).start()
                        else:
                            node.get_logger().warn("현재 방향(Yaw)이 설정되지 않아 피치를 변경할 수 없습니다. 1, 2, 3번 키를 먼저 눌러주세요.")
                
                try:
                    result_msg = node.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                    node.image_pub.publish(result_msg)
                except Exception:
                    pass
            else:
                cv2.waitKey(10)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node.is_manual_mode:
            try:
                release_compliance_ctrl()
            except:
                pass
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
