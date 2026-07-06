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
        model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best.onnx'
        if not os.path.exists(model_path):
            model_path = '/home/rokey/cobot_ws/src/ws_cobot2_pjt/src/yolov8_ws/model/best_1.onnx' # fallback
            
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
        self.FIXED_PITCH = 45.0
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
        """
        카메라가 바닥을 향해 pitch_deg 만큼 기울어져 있을 때,
        특정 픽셀(u,v)의 진짜 수직 깊이(바닥 방향으로의 직진 거리)를 계산합니다.
        """
        if Z_c <= 0 or self.intrinsics is None:
            return None
            
        theta = np.radians(pitch_deg)
        fy = self.intrinsics['fy']
        ppy = self.intrinsics['ppy']
        
        # 카메라 렌즈 기준 Y 좌표 계산
        Y_c = (v - ppy) * Z_c / fy
        
        # 수직 깊이 계산 (기하학적 보정)


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
        self.get_logger().info(f"==> {name} (각도: {target_yaw}도) 위치로 이동합니다...")
        
        pos_target = self.get_spherical_pose(target_yaw, self.FIXED_PITCH, self.FIXED_DISTANCE)
        
        try:
            if self.current_yaw is None or self.current_yaw == target_yaw:
                with self.robot_lock:
                    movel(pos_target, vel=self.VELOCITY, acc=self.ACC)
            else:
                via_yaw = (self.current_yaw + target_yaw) / 2.0
                pos_via = self.get_spherical_pose(via_yaw, self.FIXED_PITCH, self.FIXED_DISTANCE)
                with self.robot_lock:
                    movec(pos_via, pos_target, vel=self.VELOCITY, acc=self.ACC)
                
            mwait()
            self.current_yaw = target_yaw
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
                
                # 'entire'와 'hole'의 깊이를 비교하여 상대적인 높이(층수) 측정
                if node.depth_frame is not None and node.intrinsics is not None:
                    try:
                        entire_box = None
                        hole_boxes = []
                        
                        # 1. 클래스별로 바운딩 박스 분류
                        names = results[0].names
                        for box in results[0].boxes:
                            cls_name = names[int(box.cls[0])]
                            if cls_name == 'entire':
                                entire_box = box
                            elif cls_name == 'hole':
                                hole_boxes.append(box)
                                
                        # 2. 'entire' 박스가 존재하면 바닥면(가장 아랫부분) 깊이 계산
                        if entire_box is not None:
                            ex1, ey1, ex2, ey2 = entire_box.xyxy[0].cpu().numpy()
                            # entire 박스의 하단 중앙 픽셀을 탑의 바닥(테이블과 닿는 곳)으로 간주
                            eu = int((ex1 + ex2) / 2)
                            ev = int(ey2)
                            
                            eu_clp = np.clip(eu, 0, node.depth_frame.shape[1]-1)
                            ev_clp = np.clip(ev, 0, node.depth_frame.shape[0]-1)
                            
                            entire_Z = float(node.depth_frame[ev_clp, eu_clp])
                            entire_v_depth = node.get_vertical_depth(eu_clp, ev_clp, entire_Z, pitch_deg=node.FIXED_PITCH)
                            
                            if entire_v_depth is not None:
                                # 기준점(entire 바닥) 화면에 파란색 원으로 표시
                                cv2.circle(annotated_frame, (eu_clp, ev_clp), 7, (255, 0, 0), -1)
                                
                                # 3. 각 'hole'들의 상대 높이 및 층수 계산
                                for box in hole_boxes:
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    u = int((x1 + x2) / 2)
                                    v = int((y1 + y2) / 2)
                                    
                                    u_clp = np.clip(u, 0, node.depth_frame.shape[1]-1)
                                    v_clp = np.clip(v, 0, node.depth_frame.shape[0]-1)
                                    
                                    obj_Z = float(node.depth_frame[v_clp, u_clp])
                                    obj_v_depth = node.get_vertical_depth(u_clp, v_clp, obj_Z, pitch_deg=node.FIXED_PITCH)
                                    
                                    if obj_v_depth is not None:
                                        # entire 바닥 수직 깊이에서 hole의 수직 깊이를 뺌
                                        relative_height = entire_v_depth - obj_v_depth
                                        relative_height = max(0.0, relative_height)
                                        
                                        # 젠가 1층 높이를 약 15mm로 가정하고 층수(Floor) 계산
                                        floor_num = int(relative_height / 14.0) + 1
                                        
                                        # 화면에 중심점(빨간색 원)과 측정된 층수/높이 출력
                                        cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                                        cv2.putText(annotated_frame, f"F:{floor_num} ({relative_height:.1f}mm)", (u+10, v-10),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
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
                        threading.Thread(target=node.move_routine, args=(-135, 'right_back')).start()
                elif key == ord('2'):
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 이동해주세요.")
                    elif not node.is_moving:
                        threading.Thread(target=node.move_routine, args=(-45, 'right_front')).start()
                elif key == ord('3'):
                    if node.is_manual_mode:
                        node.get_logger().warn("수동 제어를 끄고 이동해주세요.")
                    elif not node.is_moving:
                        threading.Thread(target=node.move_routine, args=(45, 'left_front')).start()
                
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
