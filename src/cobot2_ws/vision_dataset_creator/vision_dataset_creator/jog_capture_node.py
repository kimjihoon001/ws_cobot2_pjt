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

# 로봇 모델 정보 (환경에 맞게 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("jog_capture_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movel, movej, get_current_posx, mwait, task_compliance_ctrl, release_compliance_ctrl
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

class JogCaptureNode(Node):
    def __init__(self):
        super().__init__("jog_capture_node")
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.save_dir = os.path.join(os.path.expanduser('~'), 'dataset_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.VELOCITY = 40
        self.ACC = 40
        self.step_size = 20.0
        self.rot_step = 5.0
        self.is_manual_mode = False

        self.get_logger().info("=========================================")
        self.get_logger().info(" Jog & Capture Node 실행됨")
        self.get_logger().info(" [키보드 조작 방법]")
        self.get_logger().info(" W/S : X축 이동 (+/-)")
        self.get_logger().info(" A/D : Y축 이동 (+/-)")
        self.get_logger().info(" Q/E : Z축 이동 (+/-)")
        self.get_logger().info(" I/K : RX 회전 (+/-)")
        self.get_logger().info(" J/L : RY 회전 (+/-)")
        self.get_logger().info(" U/O : RZ 회전 (+/-)")
        self.get_logger().info(" C   : 사진 캡처")
        self.get_logger().info(" P   : 현재 위치 출력")
        self.get_logger().info(" H   : Home 위치로 이동")
        self.get_logger().info(" M   : 수동 제어(Cockpit) 모드 On/Off 토글")
        self.get_logger().info(" ESC : 프로그램 종료")
        self.get_logger().info("=========================================")
        self.get_logger().info(" (이동 중에는 카메라 화면이 잠깐 멈출 수 있습니다)")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    def capture_and_save(self):
        if self.latest_image is not None:
            filename = os.path.join(self.save_dir, f"manual_{int(time.time())}.jpg")
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f"[캡처 성공] 파일이 저장되었습니다: {filename}")
        else:
            self.get_logger().error("카메라 이미지가 없습니다.")

def main(args=None):
    node = JogCaptureNode()
    
    try:
        while rclpy.ok():
            # 1. ROS 콜백 처리 (이미지 수신)
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # 2. 이미지 표시 및 키보드 입력 처리
            if node.latest_image is not None:
                cv2.imshow("Jog & Capture", node.latest_image)
                key = cv2.waitKey(10) & 0xFF
                
                if key == 255: # 입력 없음
                    continue
                    
                if key == 27: # ESC
                    break
                    
                if key == ord('c'):
                    node.capture_and_save()
                    continue
                    
                if key == ord('p'):
                    try:
                        pos = get_current_posx()[0]
                        node.get_logger().info(f"==> 현재 위치(posx): [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}, {pos[4]:.2f}, {pos[5]:.2f}]")
                    except Exception as e:
                        node.get_logger().error(f"위치 확인 오류: {e}")
                    continue

                if key == ord('h'):
                    node.get_logger().info("==> Home 위치로 이동합니다...")
                    try:
                        # 기본 Home 관절 각도 (M0609의 일반적인 안전 홈 위치)
                        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=node.VELOCITY, acc=node.ACC)
                        mwait()
                    except Exception as e:
                        node.get_logger().error(f"Home 이동 오류: {e}")
                    continue

                if key == ord('m'):
                    if not node.is_manual_mode:
                        node.get_logger().info("==> 수동 제어 모드 ON (로봇을 직접 움직이거나 Cockpit 버튼을 사용할 수 있습니다.)")
                        try:
                            task_compliance_ctrl()
                            node.is_manual_mode = True
                        except Exception as e:
                            node.get_logger().error(f"수동 제어 모드 켜기 오류: {e}")
                    else:
                        node.get_logger().info("==> 수동 제어 모드 OFF")
                        try:
                            release_compliance_ctrl()
                            node.is_manual_mode = False
                        except Exception as e:
                            node.get_logger().error(f"수동 제어 모드 끄기 오류: {e}")
                    continue

                # 조그 이동 처리
                try:
                    pos = get_current_posx()[0]
                except Exception:
                    continue

                target_pos = list(pos) # list 형태로 복사 (Doosan API 호환성)
                move_flag = False

                if key == ord('w'):
                    target_pos[0] += node.step_size; move_flag = True
                elif key == ord('s'):
                    target_pos[0] -= node.step_size; move_flag = True
                elif key == ord('a'):
                    target_pos[1] += node.step_size; move_flag = True
                elif key == ord('d'):
                    target_pos[1] -= node.step_size; move_flag = True
                elif key == ord('q'):
                    target_pos[2] += node.step_size; move_flag = True
                elif key == ord('e'):
                    target_pos[2] -= node.step_size; move_flag = True
                elif key == ord('i'):
                    target_pos[3] += node.rot_step; move_flag = True
                elif key == ord('k'):
                    target_pos[3] -= node.rot_step; move_flag = True
                elif key == ord('j'):
                    target_pos[4] += node.rot_step; move_flag = True
                elif key == ord('l'):
                    target_pos[4] -= node.rot_step; move_flag = True
                elif key == ord('u'):
                    target_pos[5] += node.rot_step; move_flag = True
                elif key == ord('o'):
                    target_pos[5] -= node.rot_step; move_flag = True

                if move_flag:
                    try:
                        movel(target_pos, vel=node.VELOCITY, acc=node.ACC)
                        mwait()
                    except Exception as e:
                        node.get_logger().error(f"이동 명령 오류: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
