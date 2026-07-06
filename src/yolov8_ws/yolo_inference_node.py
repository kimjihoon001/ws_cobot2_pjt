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
from scipy.spatial.transform import Rotation

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
        self.new_image_available = False # CPU 과부하 방지용 플래그
        self.depth_frame = None
        self.intrinsics = None
        
        # 캘리브레이션 데이터 로드
        calib_path = '/home/rokey/corecode/Calibration_Tutorial/T_gripper2camera.npy'
        if os.path.exists(calib_path):
            self.T_gripper2camera = np.load(calib_path)
            self.get_logger().info('T_gripper2camera 캘리브레이션 행렬 로드 성공.')
        else:
            self.get_logger().warn('캘리브레이션 파일이 없습니다. (임시 단위 행렬 적용)')
            self.T_gripper2camera = np.eye(4)
        
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
        
        # 로봇 마지막 위치 저장 변수
        self.last_pos = None
        
        # 스레드 충돌 방지용 락 (generator already executing 오류 해결)
        self.robot_lock = threading.Lock()
        
        self.get_logger().info('=========================================')
        self.get_logger().info(' YOLO Inference Node Started.')
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

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

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
        from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl, get_current_posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        sys.exit(1)

    node = YoloInferenceNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # 새 이미지가 수신되었을 때만 추론 실행 (CPU 폭주 방지)
            if node.latest_image is not None and node.new_image_available:
                node.new_image_available = False
                
                results = node.model(node.latest_image, verbose=False)
                annotated_frame = results[0].plot()
                
                if node.depth_frame is not None and node.intrinsics is not None:
                    try:
                        # 로봇이 이동 중이 아닐 때만 위치 갱신 (generator already executing 및 list out of range 방지)
                        if not node.is_moving:
                            with node.robot_lock:
                                pos_res = get_current_posx()
                                if pos_res and len(pos_res) > 0 and len(pos_res[0]) >= 6:
                                    node.last_pos = pos_res[0]
                        
                        if node.last_pos is not None:
                            T_base2gripper = node.get_robot_pose_matrix(*node.last_pos)
                            T_base2camera = T_base2gripper @ node.T_gripper2camera
                            
                            for box in results[0].boxes:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                u = int((x1 + x2) / 2)
                                v = int((y1 + y2) / 2)
                                
                                u_clp = np.clip(u, 0, node.depth_frame.shape[1]-1)
                                v_clp = np.clip(v, 0, node.depth_frame.shape[0]-1)
                                
                                Z_c = float(node.depth_frame[v_clp, u_clp])
                                
                                if Z_c > 0:
                                    fx = node.intrinsics['fx']
                                    fy = node.intrinsics['fy']
                                    ppx = node.intrinsics['ppx']
                                    ppy = node.intrinsics['ppy']
                                    
                                    X_c = (u_clp - ppx) * Z_c / fx
                                    Y_c = (v_clp - ppy) * Z_c / fy
                                    
                                    P_camera = np.array([X_c, Y_c, Z_c, 1.0])
                                    P_base = T_base2camera @ P_camera
                                    
                                    base_z = P_base[2]
                                    
                                    cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                                    cv2.putText(annotated_frame, f"Z:{base_z:.1f}mm", (u+10, v-10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    except Exception as e:
                        node.get_logger().error(f"3D 좌표 계산 오류: {e}")
                
                cv2.imshow("YOLOv8 Inference & Control", annotated_frame)
                key = cv2.waitKey(20) & 0xFF # 20ms 대기 (초당 50프레임 제한)
                
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
                # 새 이미지가 없을 경우 리소스를 위해 약간 대기
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
