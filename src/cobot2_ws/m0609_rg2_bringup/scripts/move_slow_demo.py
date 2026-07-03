#!/usr/bin/env python3
# 실제/가상 로봇에 MoveIt으로 관절 목표를 "plan + execute"하는 최소 데모.
# CMakeLists.txt에 미등록 상태라 ros2 run 불가 — python3로 직접 실행:
#   python3 move_slow_demo.py
#
# 사전 조건:
#   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<IP> port:=<PORT>
#   ros2 launch m0609_rg2_moveit movegroup_only.launch.py
#
# 안전을 위해 속도/가속도를 SPEED_SCALE(기본 5%)로 강하게 제한하고,
# 목표 관절값은 현재 위치에서 아주 작게 벗어난 값으로 시작할 것을 권장.

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions, RobotState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# 목표 관절 각도 (rad). 실행 전에 반드시 현재 로봇 자세 기준으로 값을 확인/수정할 것.
TARGET_JOINTS = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

# 최대 속도/가속도 스케일 (0.0~1.0). 낮을수록 천천히 움직임.
SPEED_SCALE = 0.05


class SlowMover(Node):
    def __init__(self):
        super().__init__('move_slow_demo')
        self._ac = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('move_group 대기중...')
        self._ac.wait_for_server()
        self.get_logger().info('연결됨')

    def move_to(self, joint_positions, speed_scale):
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
        req.start_state.is_diff = True  # 현재 실제 로봇 관절 상태를 시작점으로 사용
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = speed_scale
        req.max_acceleration_scaling_factor = speed_scale

        opts = PlanningOptions()
        opts.plan_only = False  # 계획 성공 시 바로 실행

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        self.get_logger().info(f'목표: {joint_positions}, 속도 스케일: {speed_scale}')
        send_future = self._ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('목표가 move_group에서 거부됨')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('실행 완료')
            return True
        else:
            self.get_logger().error(f'실패: error_code={result.error_code.val}')
            return False


def main():
    rclpy.init()
    node = SlowMover()
    ok = node.move_to(TARGET_JOINTS, SPEED_SCALE)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
