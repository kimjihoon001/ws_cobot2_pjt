#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

usage = """
=====================================================
            가상 손(장애물) 위치 제어 스크립트
=====================================================
조작 방법 (키를 누르면 즉시 위치가 변경되어 발행됩니다):

    [X 축 조작]
    w : X 증가 (+0.05m)  /  s : X 감소 (-0.05m)

    [Y 축 조작]
    a : Y 증가 (+0.05m)  /  d : Y 감소 (-0.05m)

    [Z 축 조작]
    q : Z 증가 (+0.05m)  /  e : Z 감소 (-0.05m)

    [매크로 키]
    space : 장애물 치우기 (Z = -2.0m로 이동시켜 회피 해제)
    r     : 장애물 경로 중앙에 배치 (로봇 경로 간섭 위치)

    CTRL-C : 종료
=====================================================
"""


class HandPublisher(Node):
    def __init__(self):
        super().__init__('hand_publisher')
        self.pub = self.create_publisher(Point, '/hand_position', 10)
        self.x = 0.4
        self.y = -0.1
        self.z = 0.3  # 기본 간섭 가능 위치
        self.publish_pos()

    def publish_pos(self):
        msg = Point()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        self.pub.publish(msg)
        print(f"현재 장애물 위치 발행 중: X={self.x:+.2f}, Y={self.y:+.2f}, Z={self.z:+.2f} (단위: m)")


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = HandPublisher()

    print(usage)
    node.publish_pos()

    try:
        while True:
            # 키보드 입력 받기 (non-blocking)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = None
            if rlist:
                key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            if key:
                if key == 'w':
                    node.x += 0.05
                elif key == 's':
                    node.x -= 0.05
                elif key == 'a':
                    node.y += 0.05
                elif key == 'd':
                    node.y -= 0.05
                elif key == 'q':
                    node.z += 0.05
                elif key == 'e':
                    node.z -= 0.05
                elif key == ' ':
                    node.z = -2.0  # 장애물 완전히 치우기
                elif key == 'r':
                    node.x = 0.4
                    node.y = -0.1
                    node.z = 0.3  # 중앙 복귀
                elif key == '\x03':  # Ctrl+C
                    break
                
                # 값 변경 시마다 토픽 발행
                node.publish_pos()

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
