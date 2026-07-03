import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    RobotState,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self._ac = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('move_group 대기중...')
        self._ac.wait_for_server()
        self.get_logger().info('연결!')
    
    def plan(self):
        start = RobotState()
        start.joint_state.name = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        start.joint_state.position = [0.0, -0.5, 1.5, 0.0, 1.0, 0.0]

        # jc = JointConstraint()
        # jc.joint_name = 'joint_1'
        # jc.position = 1.0
        # jc.tolerance_above = 0.01
        # jc.tolerance_below = 0.01
        # jc.weight = 1.0

        # goal_constraints = Constraints()
        # goal_constraints.joint_constraints = [jc]

        target_pose = Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.2
        target_pose.position.z = 0.5
        target_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [target_pose]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]

        req = MotionPlanRequest()
        req.group_name = 'manipulator'
        req.start_state = start
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 5.0 
        
        opts = PlanningOptions()
        opts.plan_only = True

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        send_future = self._ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('플래닝 성공!')
            traj = result.planned_trajectory.joint_trajectory
            print(f'경유점 수: {len(traj.points)}')
            for i, pt in enumerate(traj.points):
                print(f'  [{i}] {[round(p,3) for p in pt.positions]}')
        else:
            self.get_logger().error(f'실패: {result.error_code.val}')

def main():
    rclpy.init()
    node = Planner()
    node.plan()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()