import os
import time
import sys
import threading
import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import DR_init

# 로봇 모델 정보 (상황에 맞게 수정 가능)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# DSR_ROBOT2 라이브러리를 쓰기 위해서는 rclpy.init()과 노드 설정이 먼저 필요합니다.
rclpy.init()
dsr_node = rclpy.create_node("vision_dataset_capture_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, movec, mwait, set_tcp
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

try:
    from pick_and_place_text.onrobot import RG
    gripper = RG("rg2", "192.168.1.1", "502")
except ImportError as e:
    print(f"Error importing gripper (RG): {e}")
    gripper = None

class AutoDatasetCaptureNode(Node):
    def __init__(self):
        super().__init__("auto_dataset_capture_node")
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 리얼센스 카메라 토픽 (환경에 맞게 수정하세요. 보통 /camera/color/image_raw 입니다.)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 이미지가 저장될 경로 (홈 디렉토리 아래 dataset_images 폴더)
        self.save_dir = os.path.join(os.path.expanduser('~'), 'dataset_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"데이터셋 저장 폴더 생성됨: {self.save_dir}")

        # === 촬영 대상 물체의 실제 좌표 ===
        # 예시: [x, y, z] (단위: mm)
        # 조그 노드로 물체 중심점의 좌표를 찾아서 입력해주세요.
        self.OBJECT_POS = [367.0, 3.0, 30.0]
        
        # 물체로부터 카메라를 얼마나 띄울지 (구형 궤적의 반지름, 단위: mm)
        self.DISTANCE = 320.0

        # 로봇 끝단(TCP)과 실제 카메라 렌즈 사이의 Y축 거리 차이 (단위: mm)
        # 카메라가 로봇팔 끝에서 Y축 방향(측면)으로 75mm 떨어져 있으므로 75.0을 줍니다.
        self.CAMERA_Y_OFFSET = 75.0

        # 카메라를 기울일 각도 (수직 위에서부터 내려다보는 각도, 단위: degree)
        # 예) 45.0 이면 비스듬하게 측면 위에서 물체를 내려다보게 됩니다.
        self.PITCH_ANGLE = 45.0
        
        self.VELOCITY = 60
        self.ACC = 60

        self.get_logger().info("Dataset Capture Node가 초기화되었습니다.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

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
            
            # 카메라가 완전히 수직 아래를 볼 경우 (y_c의 길이가 0에 가까움)
            # 로봇 베이스에서 타겟을 향하는 팔의 방향을 기준으로 자연스러운 축 설정
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
        
        # Rotation Matrix -> ZYZ Euler Angles
        beta = np.arctan2(np.sqrt(R[2,0]**2 + R[2,1]**2), R[2,2])
        if np.abs(beta) < 1e-6:
            alpha = 0.0
            gamma = np.arctan2(-R[0,1], R[0,0])
        else:
            alpha = np.arctan2(R[1,2], R[0,2])
            gamma = np.arctan2(R[2,1], -R[2,0])
            
        # 카메라 위치(c)에서 카메라의 Y축(y_c) 방향으로 y_offset만큼 뺀 위치가 TCP 위치
        tcp_pos = c - y_offset * y_c
        return tcp_pos.tolist() + np.degrees([alpha, beta, gamma]).tolist()

    def capture_and_save(self, position_name):
        # 로봇이 멈춘 후 카메라 화면이 안정화되기를 잠깐 기다림
        time.sleep(1.0)
        
        if self.latest_image is not None:
            # 파일 이름에 현재 시간을 붙여서 중복 방지
            filename = os.path.join(self.save_dir, f"{position_name}_{int(time.time())}.jpg")
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f"[{position_name}] 방향 이미지 저장 완료: {filename}")
        else:
            self.get_logger().error(f"[{position_name}] 방향의 이미지를 수신하지 못했습니다. 카메라 토픽을 확인하세요.")

    def get_spherical_pose(self, angle_deg, pitch_deg, distance):
        target = self.OBJECT_POS[:3]
        d = distance
        pitch = np.radians(pitch_deg)
        rad = np.radians(angle_deg)
        r_xy = d * np.sin(pitch)
        z = target[2] + d * np.cos(pitch)
        
        cam_pos = [target[0] + r_xy * np.cos(rad), target[1] + r_xy * np.sin(rad), z]
        return self.calculate_tcp_pose(cam_pos, target, self.CAMERA_Y_OFFSET)

    def capture_routine(self):
        self.get_logger().info("데이터셋 촬영 루틴을 시작합니다...")
        
        if gripper:
            self.get_logger().info("시야 확보를 위해 그리퍼를 폅니다.")
            gripper.open_gripper()
            time.sleep(1.0)
            
        self.get_logger().info("관절 꼬임을 방지하기 위해 먼저 Home 위치로 이동하여 자세를 풉니다.")
        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=self.VELOCITY, acc=self.ACC)
        mwait()
        
        if sum(self.OBJECT_POS) == 0:
            self.get_logger().error("OBJECT_POS 좌표가 입력되지 않았습니다. 코드를 열어 물체의 실제 좌표를 입력해주세요.")
            return

        # 거리(220~320) 5단계, 피치 각도(30~60) 5단계 = 총 25개의 조합
        distances = np.linspace(250.0, 320.0, 5)
        pitches = np.linspace(30.0, 60.0, 5)
        
        # 3가지 방향 (로봇 뒤쪽 충돌을 피하기 위해 물체 앞쪽으로 도는 궤적, back_right부터 시작)
        yaw_angles_fwd = [-135, -45, 45]
        yaw_names_fwd = ["back_right", "right_front", "front_left"]
        
        home_joint = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 안전을 위해 최초 한 번은 물체 바로 위(Top)로 이동
        top_pos = self.get_spherical_pose(-135, 0, 320.0)
        self.get_logger().info("안전하게 물체 위(Top)로 이동합니다.")
        movel(top_pos, vel=self.VELOCITY, acc=self.ACC)
        mwait()
        
        count = 1
        sweep_fwd = True
        
        for d in distances:
            for p in pitches:
                self.get_logger().info(f"Sweep 시작 - 거리: {d:.1f}mm, 각도: {p:.1f}도")
                
                # 와이퍼처럼 스윕 방향을 번갈아가며(지그재그) 수행하여 등 뒤로 넘어가는 움직임을 방지
                current_yaws = yaw_angles_fwd if sweep_fwd else yaw_angles_fwd[::-1]
                current_names = yaw_names_fwd if sweep_fwd else yaw_names_fwd[::-1]
                
                prev_yaw = None
                for idx, yaw in enumerate(current_yaws):
                    name = current_names[idx]
                    pos_target = self.get_spherical_pose(yaw, p, d)
                    
                    if prev_yaw is None:
                        # 패스의 첫 번째 지점은 직선 이동
                        self.get_logger().info(f"로봇팔을 [{name}] 위치로 이동합니다. (거리:{d:.0f}, 각도:{p:.0f})")
                        movel(pos_target, vel=self.VELOCITY, acc=self.ACC)
                    else:
                        # 완벽한 구면 궤적 유지를 위한 movec (Via 포인트를 절반 각도로 계산)
                        via_yaw = (prev_yaw + yaw) / 2.0
                        pos_via = self.get_spherical_pose(via_yaw, p, d)
                        movec(pos_via, pos_target, vel=self.VELOCITY, acc=self.ACC)
                        
                    mwait()
                    
                    # 카메라의 현재 높이(Z) 계산
                    height = self.OBJECT_POS[2] + d * np.cos(np.radians(p))
                    
                    # 저장될 파일명에 현재 방향(name), 회전 각도(yaw), 피치 각도(pitch), 높이(h), 거리(d) 포함
                    filename_prefix = f"img_{count:03d}_{name}_yaw{int(yaw)}_pitch{int(p)}_h{int(height)}_d{int(d)}"
                    self.capture_and_save(filename_prefix)
                    
                    count += 1
                    prev_yaw = yaw
                
                # 다음 루프에서는 역방향으로 훑음
                sweep_fwd = not sweep_fwd
            
        self.get_logger().info("75개의 데이터셋 촬영이 모두 완료되었습니다.")
        
        # 촬영 종료 후 안전하게 Home 위치로 복귀
        self.get_logger().info("모든 촬영이 끝났습니다. 안전을 위해 Home 위치로 복귀합니다.")
        movej(home_joint, vel=self.VELOCITY, acc=self.ACC)
        mwait()
            
        self.get_logger().info("데이터셋 촬영 루틴이 모두 완료되었습니다.")


def main(args=None):
    # DR_init 설정 전에 노드를 생성했으므로 여기서는 추가 초기화 불필요
    try:
        set_tcp("2FG_TCP")
        print("TCP successfully set to 2FG_TCP")
    except Exception as e:
        print(f"Warning: Failed to set TCP to 2FG_TCP: {e}")
        
    node = AutoDatasetCaptureNode()
    
    # 로봇 이동(mwait)은 블로킹 함수이므로, 이미지 콜백이 멈추지 않도록 별도의 스레드에서 실행
    routine_thread = threading.Thread(target=node.capture_routine)
    routine_thread.start()
    
    try:
        # 메인 스레드에서는 별도의 Executor를 만들어 토픽을 수신 (DSR_ROBOT2 내부 로직과 충돌 방지)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        routine_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
