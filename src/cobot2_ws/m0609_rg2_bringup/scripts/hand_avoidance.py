#!/usr/bin/env python3
import sys
import time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
    PlanningScene,
    CollisionObject
)
from shape_msgs.msg import SolidPrimitive

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Pose A: 좌측 위치 (라디안 단위)
POSE_A = [-0.6, 0.0, 1.5708, 0.0, 1.5708, 0.0]
# Pose B: 우측 위치 (라디안 단위)
POSE_B = [0.6, 0.0, 1.5708, 0.0, 1.5708, 0.0]

SPEED_SCALE = 0.1  # 속도 제한 (10%)


class HandAvoidanceNode(Node):
    def __init__(self):
        super().__init__('hand_avoidance')

        # 1. /hand_position 토픽 구독 (사람 손 위치 입력)
        self.subscription = self.create_subscription(
            Point,
            '/hand_position',
            self.hand_position_callback,
            10
        )

        # 2. PlanningScene 퍼블리셔 생성 (장애물 동적 등록)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # 3. MoveGroup 액션 클라이언트 생성
        self._ac = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('move_group 액션 서버 연결 대기 중...')
        self._ac.wait_for_server()
        self.get_logger().info('move_group 액션 서버 연결 완료!')

        # 가상 손 위치 저장용 변수
        self.hand_x = 0.0
        self.hand_y = 0.0
        self.hand_z = -2.0  # 처음에는 로봇 작동 범위 밖(바닥 아래)에 위치시킴

        # 4. 반복 왕복 운동을 처리할 스레드 기동 (구독 콜백의 병렬 처리를 위해 필수)
        self.loop_thread = threading.Thread(target=self.movement_loop)
        self.loop_thread.daemon = True
        self.loop_thread.start()

    def hand_position_callback(self, msg):
        """가상 손 위치가 변경되면 MoveIt의 Planning Scene에 장애물로 등록"""
        self.hand_x = msg.x
        self.hand_y = msg.y
        self.hand_z = msg.z
        self.get_logger().info(f'손 감지 위치 변경 수신: x={self.hand_x:.2f}, y={self.hand_y:.2f}, z={self.hand_z:.2f}')

        # PlanningScene에 구체(Sphere) 장애물 추가
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'human_hand'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        # 반지름 5cm (0.05m) 크기의 가상 장애물 구체 설정
        sphere.dimensions = [0.05]

        pose = Pose()
        pose.position.x = self.hand_x
        pose.position.y = self.hand_y
        pose.position.z = self.hand_z
        pose.orientation.w = 1.0

        obj.primitives = [sphere]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        self.scene_pub.publish(scene)

    def move_to_joints(self, joint_positions):
        """지정된 관절 목표 값으로 MoveIt을 통해 plan + execute 수행"""
        goal_constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, joint_positions):
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
        req.start_state.is_diff = True  # 현재 실제 로봇 상태를 출발지로 지정
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = SPEED_SCALE
        req.max_acceleration_scaling_factor = SPEED_SCALE

        opts = PlanningOptions()
        opts.plan_only = False  # 계획 수립 후 즉시 실행

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        self.get_logger().info('MoveGroup Goal 전송 중...')
        send_future = self._ac.send_goal_async(goal_msg)
        
        # 스레드 루프 내에서 비동기 결과 대기
        while not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('경로 계획 요청이 거부되었습니다.')
            return False

        self.get_logger().info('경로 계획 수락됨. 실행 중...')
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('목표 위치 이동 완료!')
            return True
        else:
            self.get_logger().error(f'이동 실패: 에러 코드={result.error_code.val}')
            return False

    def movement_loop(self):
        """Pose A와 Pose B를 번갈아가며 반복 왕복하는 스레드 루프"""
        time.sleep(2.0)  # 초기 안정화를 위해 잠시 대기
        self.get_logger().info('반복 왕복 모션 루프를 시작합니다.')

        target_toggle = True
        while rclpy.ok():
            if target_toggle:
                self.get_logger().info('>>> Pose A로 이동 계획 중...')
                success = self.move_to_joints(POSE_A)
            else:
                self.get_logger().info('>>> Pose B로 이동 계획 중...')
                success = self.move_to_joints(POSE_B)

            if success:
                target_toggle = not target_toggle
            else:
                self.get_logger().warn('이동 계획에 실패하여 재시도합니다. (장애물이 복귀 경로를 막고 있을 수 있습니다.)')

            time.sleep(1.5)  # 도착 후 잠시 대기


def main(args=None):
    rclpy.init(args=args)
    node = HandAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 인터럽트로 종료 중...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
