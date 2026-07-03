import os
import time
import sys
import threading
import cv2
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
    from DSR_ROBOT2 import movej, movel, mwait
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

class DatasetCaptureNode(Node):
    def __init__(self):
        super().__init__("dataset_capture_node")
        
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

        # === 사용자가 직접 값을 입력할 위치 변수 ===
        # 예시: [x, y, z, rx, ry, rz] (단위: mm, degree)
        self.FRONT_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.BACK_POS  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.LEFT_POS  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.RIGHT_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.OVER_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.VELOCITY = 60
        self.ACC = 60

        self.get_logger().info("Dataset Capture Node가 초기화되었습니다.")

    def image_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV Mat으로 변환
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

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

    def capture_routine(self):
        self.get_logger().info("데이터셋 촬영 루틴을 시작합니다...")
        
        positions = {
            "front": self.FRONT_POS,
            "back": self.BACK_POS,
            "left": self.LEFT_POS,
            "right": self.RIGHT_POS
        }
        
        for name, pos in positions.items():
            # 사용자가 좌표를 입력하지 않았으면(모두 0이면) 건너뜀
            if sum(pos) == 0:
                self.get_logger().warn(f"[{name}] 방향의 좌표가 입력되지 않았습니다(모두 0). 스킵합니다.")
                continue
                
            self.get_logger().info(f"로봇팔을 [{name}] 위치로 이동합니다: {pos}")
            
            # 로봇팔 이동 (movel은 Tool Center Point가 직선으로 이동)
            movel(pos, vel=self.VELOCITY, acc=self.ACC)
            mwait() # 로봇이 목표 지점에 도달할 때까지 대기
            
            # 이미지 캡처 및 저장
            self.capture_and_save(name)
            
        self.get_logger().info("데이터셋 촬영 루틴이 모두 완료되었습니다.")


def main(args=None):
    # DR_init 설정 전에 노드를 생성했으므로 여기서는 추가 초기화 불필요
    node = DatasetCaptureNode()
    
    # 로봇 이동(mwait)은 블로킹 함수이므로, 이미지 콜백이 멈추지 않도록 별도의 스레드에서 실행
    routine_thread = threading.Thread(target=node.capture_routine)
    routine_thread.start()
    
    try:
        # 메인 스레드에서는 rclpy.spin을 통해 토픽을 계속 수신
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        routine_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
