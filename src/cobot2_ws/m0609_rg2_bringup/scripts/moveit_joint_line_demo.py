#!/usr/bin/env python3
"""MoveIt joint-space demo from a fixed start posture to a fixed target posture.

Run after bringup and move_group are already running:
  ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=<ROBOT_IP>
  ros2 launch m0609_rg2_moveit movegroup_only.launch.py
  python3 src/cobot2_ws/m0609_rg2_bringup/scripts/moveit_joint_line_demo.py
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    RobotState,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GROUP_NAME = 'manipulator'

START_JOINTS_DEG = [-39.51, -21.94, 115.68, -0.33, 85.92, -39.27]
TARGET_JOINTS_DEG = [-67.14, 10.59, 87.35, -0.11, 81.50, -66.79]

SPEED_SCALE = 0.10
ACCEL_SCALE = 0.10
START_TOLERANCE_DEG = 20.0
PLANNING_FRAME = 'base_link'
EEF_LINK = 'rg2_tcp'
SCENE_OBJECT_IDS = ['human_hand', 'hammer', 'screwdriver']
JOINT_LIMITS_RAD = {
    'joint_1': (-3.14, 3.14),
    'joint_2': (-1.6581, 1.6581),
    'joint_3': (-2.094395, 2.094395),
    'joint_4': (-3.14, 3.14),
    'joint_5': (-2.3562, 2.3562),
    'joint_6': (-3.14, 3.14),
}

ERROR_CODE_NAMES = {
    MoveItErrorCodes.SUCCESS: 'SUCCESS',
    MoveItErrorCodes.FAILURE: 'FAILURE',
    MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
    MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
    MoveItErrorCodes.PREEMPTED: 'PREEMPTED',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    MoveItErrorCodes.START_STATE_INVALID: 'START_STATE_INVALID',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
    MoveItErrorCodes.GOAL_STATE_INVALID: 'GOAL_STATE_INVALID',
    MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE: 'UNRECOGNIZED_GOAL_TYPE',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
    MoveItErrorCodes.INVALID_LINK_NAME: 'INVALID_LINK_NAME',
    MoveItErrorCodes.INVALID_OBJECT_NAME: 'INVALID_OBJECT_NAME',
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'FRAME_TRANSFORM_FAILURE',
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'COLLISION_CHECKING_UNAVAILABLE',
    MoveItErrorCodes.ROBOT_STATE_STALE: 'ROBOT_STATE_STALE',
    MoveItErrorCodes.SENSOR_INFO_STALE: 'SENSOR_INFO_STALE',
    MoveItErrorCodes.COMMUNICATION_FAILURE: 'COMMUNICATION_FAILURE',
    MoveItErrorCodes.CRASH: 'CRASH',
    MoveItErrorCodes.ABORT: 'ABORT',
    MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
}


def deg_list_to_rad(values_deg: List[float]) -> List[float]:
    return [math.radians(value) for value in values_deg]


def rad_list_to_deg(values_rad: List[float]) -> List[float]:
    return [math.degrees(value) for value in values_rad]


class MoveItJointLineDemo(Node):
    def __init__(self, force: bool):
        super().__init__('moveit_joint_line_demo')
        self.force = force
        self.current_joints: Optional[Dict[str, float]] = None
        self.start_joints: Optional[List[float]] = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self._collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self._attached_collision_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)

        self._move_ac = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('move_group 대기중...')
        self._move_ac.wait_for_server()
        self.get_logger().info('move_group 연결 완료')

    def _joint_state_cb(self, msg: JointState) -> None:
        joints = {}
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                joints[name] = position
        if len(joints) == len(JOINT_NAMES):
            self.current_joints = joints

    def wait_for_current_joints(self, timeout_sec: float = 3.0) -> Optional[List[float]]:
        end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joints is not None:
                return [self.current_joints[name] for name in JOINT_NAMES]
        return None

    def clear_known_scene_objects(self) -> None:
        self.get_logger().info(f'Planning scene 정리: {SCENE_OBJECT_IDS}')
        for _ in range(5):
            for object_id in SCENE_OBJECT_IDS:
                attached = AttachedCollisionObject()
                attached.link_name = EEF_LINK
                attached.object.id = object_id
                attached.object.operation = CollisionObject.REMOVE

                world = CollisionObject()
                world.header.frame_id = PLANNING_FRAME
                world.id = object_id
                world.operation = CollisionObject.REMOVE

                self._attached_collision_pub.publish(attached)
                self._collision_pub.publish(world)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self.get_logger().info('Planning scene 정리 완료')

    def check_start_position(self) -> bool:
        current = self.wait_for_current_joints()
        if current is None:
            self.get_logger().warn('/joint_states에서 현재 조인트를 못 읽었습니다. MoveIt 현재 상태 기준으로 진행합니다.')
            return True

        current_deg = rad_list_to_deg(current)
        self.start_joints = current
        out_of_bounds = []
        for name, position in zip(JOINT_NAMES, current):
            lower, upper = JOINT_LIMITS_RAD[name]
            if position < lower or position > upper:
                out_of_bounds.append(
                    f'{name}={math.degrees(position):.2f}deg '
                    f'(limit {math.degrees(lower):.2f}~{math.degrees(upper):.2f}deg)'
                )
        if out_of_bounds:
            self.get_logger().error('현재 조인트가 MoveIt limit 밖입니다: ' + ', '.join(out_of_bounds))
            self.get_logger().error('joint_limits.yaml 수정 후 move_group을 재시작해야 반영됩니다.')
            return False

        diffs = [abs(a - b) for a, b in zip(current_deg, START_JOINTS_DEG)]
        max_diff = max(diffs)
        self.get_logger().info(f'현재 조인트(deg): {[round(v, 2) for v in current_deg]}')
        self.get_logger().info(f'시작값과 최대 차이: {max_diff:.2f} deg')

        if max_diff <= START_TOLERANCE_DEG:
            return True

        message = (
            f'현재 자세가 지정 시작값과 {max_diff:.2f} deg 차이납니다. '
            f'그래도 실행하려면 --force 옵션을 사용하세요.'
        )
        if self.force:
            self.get_logger().warn(message)
            return True

        self.get_logger().error(message)
        return False

    def move_to_target(self) -> bool:
        if self.start_joints is None:
            self.start_joints = self.wait_for_current_joints()
        if self.start_joints is None:
            self.get_logger().error('MoveIt 시작 상태로 넣을 현재 조인트를 읽지 못했습니다.')
            return False

        target_joints = deg_list_to_rad(TARGET_JOINTS_DEG)

        goal_constraints = Constraints()
        for name, position in zip(JOINT_NAMES, target_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = math.radians(0.5)
            joint_constraint.tolerance_below = math.radians(0.5)
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)

        request = MotionPlanRequest()
        request.group_name = GROUP_NAME
        request.start_state = RobotState()
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = self.start_joints
        request.start_state.is_diff = False
        request.goal_constraints = [goal_constraints]
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = SPEED_SCALE
        request.max_acceleration_scaling_factor = ACCEL_SCALE

        options = PlanningOptions()
        options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        self.get_logger().info(f'목표 조인트(deg): {TARGET_JOINTS_DEG}')
        self.get_logger().info(f'속도/가속도 스케일: {SPEED_SCALE:.2f}/{ACCEL_SCALE:.2f}')

        send_future = self._move_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveGroup 목표가 거부되었습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('MoveIt 실행 완료')
            return True

        error_name = ERROR_CODE_NAMES.get(result.error_code.val, 'UNKNOWN')
        self.get_logger().error(f'MoveIt 실행 실패: error_code={result.error_code.val} ({error_name})')
        return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--force', action='store_true', help='현재 조인트가 시작값과 달라도 실행')
    args = parser.parse_args()

    rclpy.init()
    node = MoveItJointLineDemo(force=args.force)
    try:
        node.clear_known_scene_objects()
        ok = node.check_start_position() and node.move_to_target()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
