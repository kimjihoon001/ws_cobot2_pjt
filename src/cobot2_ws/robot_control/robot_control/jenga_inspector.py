import os
import sys
import time
import json
import sqlite3
import threading

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from od_msg.srv import SrvDepthPosition

from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, PointStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
import tf2_geometry_msgs
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState
)
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Initialize DSR Node
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40  # Slower for safety during inspection
SCAN_DISTANCE = 250.0   # mm from Jenga block center
SCAN_HEIGHT = 100.0     # mm height offset above Jenga center
CAMERA_Y_OFFSET = 75.0  # mm offset of camera from TCP

# Jenga Tower Dimensions ( 6 layers: 75mm x 75mm x 84mm )
JENGA_WIDTH = 75.0   # mm (3 blocks * 25mm = 75mm)
JENGA_DEPTH = 75.0   # mm (75mm)
JENGA_HEIGHT = 84.0  # mm (6 layers * 14mm = 84mm)

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

if not rclpy.ok():
    rclpy.init()

dsr_node = rclpy.create_node("jenga_inspector_dsr_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movejx, movel, mwait, get_current_posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Define expected configuration for PASS/FAIL
# Face 0: Front, Face 1: Right, Face 2: Back, Face 3: Left
# You can customize these expected hole counts for your Jenga assembly.
EXPECTED_FEATURES = {
    0: {"smallhole": 2, "longhole": 1},
    1: {"smallhole": 1, "longhole": 2},
    2: {"smallhole": 2, "longhole": 1},
    3: {"smallhole": 1, "longhole": 2}
}


class JengaInspectorNode(Node):
    def __init__(self):
        super().__init__("jenga_inspector")
        self.package_path = get_package_share_directory("robot_control")
        self.init_database()

        # Helper node for blocking action and service calls to prevent executor deadlock
        self.action_node = rclpy.create_node("jenga_inspector_action_node")
        self.ikin_client = self.action_node.create_client(GetPositionIK, '/compute_ik')
        self._ac = ActionClient(self.action_node, MoveGroup, '/move_action')

        # Clients for perception (created on action_node to bypass executor deadlock)
        self.get_position_client = self.action_node.create_client(
            SrvDepthPosition, "/get_jenga_position"
        )
        self.detect_features_client = self.action_node.create_client(
            Trigger, "/detect_jenga_features"
        )
        
        self.action_executor = rclpy.executors.SingleThreadedExecutor()
        self.action_executor.add_node(self.action_node)
        self.action_thread = threading.Thread(target=self.action_executor.spin, daemon=True)
        self.action_thread.start()

        self.get_logger().info("Waiting for MoveGroup Action server...")
        self._ac.wait_for_server()
        self.get_logger().info("MoveGroup Action server connected.")

        # TF Listener to get current robot pose without blocking DSR services
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for RViz Planning Scene (MoveIt)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Service for triggering inspection
        self.create_service(
            Trigger,
            '/run_jenga_inspection',
            self.handle_run_jenga_inspection
        )
        
        # Hand avoidance parameters & states
        self.hand_detected = False
        self.current_hand_pos = None
        self.active_goal_handle = None
        self.in_evasion = False
        self.last_log_time = 0.0
        
        # Subscribe to hand position topic with a ReentrantCallbackGroup to prevent service call deadlocks
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.hand_sub = self.create_subscription(
            Point,
            '/hand_position',
            self.hand_position_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("JengaInspectorNode initialized. Service '/run_jenga_inspection' is ready.")

    def hand_position_callback(self, msg):
        """Processes hand position, filters by robot workspace ROI, and updates obstacle."""
        is_clear_signal = msg.z < -1.0
        
        # 타이트한 작업 공간 ROI 필터 (단위: 미터)
        # X: 앞쪽 22cm ~ 55cm, Y: 좌우 -30cm ~ 30cm, Z: 높이 -2cm ~ 45cm
        in_roi = False
        if not is_clear_signal:
            in_roi = (0.22 <= msg.x <= 0.55) and (-0.30 <= msg.y <= 0.30) and (-0.02 <= msg.z <= 0.45)
            
        if is_clear_signal or not in_roi:
            if self.hand_detected:
                self.hand_detected = False
                self.current_hand_pos = None
                self.clear_hand_obstacle()
                self.get_logger().info("Hand cleared (left FOV or exited workspace ROI).")
        else:
            self.hand_detected = True
            self.current_hand_pos = [msg.x, msg.y, msg.z]
            now = time.time()
            if now - self.last_log_time > 1.0:
                self.get_logger().info(f"Hand detected inside active ROI: x={msg.x:.3f}m, y={msg.y:.3f}m, z={msg.z:.3f}m")
                self.last_log_time = now
            self.update_hand_obstacle(msg.x, msg.y, msg.z)

    def update_hand_obstacle(self, x, y, z):
        """Spawns/Updates hand obstacle in the MoveIt planning scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'human_hand'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.025] # 2.5cm safety sphere radius (5cm diameter)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives = [sphere]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
        self.scene_pub.publish(scene)

    def clear_hand_obstacle(self):
        """Removes the hand obstacle from the MoveIt planning scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'human_hand'
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
        self.scene_pub.publish(scene)
        self.get_logger().info("Cleared hand obstacle from planning scene.")

    def init_database(self):
        """Creates SQLite database and inspection_logs table if not exists."""
        db_dir = os.path.join(self.package_path, "resource")
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)
        self.db_path = os.path.join(db_dir, "inspection.db")
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS inspection_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                result TEXT,
                confidence REAL
            )
        """)
        conn.commit()
        conn.close()
        self.get_logger().info(f"Database initialized at {self.db_path}")

    def solve_ik(self, pose):
        """Solves inverse kinematics for a Cartesian pose using MoveIt compute_ik.
        Accepts [x,y,z,rx,ry,rz] (Euler ZYZ) or [x,y,z,qx,qy,qz,qw] (quaternion).
        Returns joint angles in degrees, or None if failed."""
        if not self.ikin_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK service /compute_ik not available")
            return None
            
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'manipulator'
        ik_req.ik_link_name = 'tool0' # Solve IK for tool0 flange
        ik_req.pose_stamped.header.frame_id = 'base_link'
        
        # Position in meters
        ik_req.pose_stamped.pose.position.x = pose[0] / 1000.0
        ik_req.pose_stamped.pose.position.y = pose[1] / 1000.0
        ik_req.pose_stamped.pose.position.z = pose[2] / 1000.0
        
        if len(pose) == 7:
            # Pose with quaternion orientation
            ik_req.pose_stamped.pose.orientation.x = pose[3]
            ik_req.pose_stamped.pose.orientation.y = pose[4]
            ik_req.pose_stamped.pose.orientation.z = pose[5]
            ik_req.pose_stamped.pose.orientation.w = pose[6]
        else:
            # Pose with Euler angles ZYZ in degrees
            r = Rotation.from_euler('ZYZ', [pose[3], pose[4], pose[5]], degrees=True)
            q = r.as_quat()
            ik_req.pose_stamped.pose.orientation.x = q[0]
            ik_req.pose_stamped.pose.orientation.y = q[1]
            ik_req.pose_stamped.pose.orientation.z = q[2]
            ik_req.pose_stamped.pose.orientation.w = q[3]
            
        # Seed the solver using the initial pose (in radians) to avoid local minima or singularities
        joint_state = JointState()
        joint_state.name = JOINT_NAMES
        joint_state.position = [np.radians(angle) for angle in [-44.26, 18.14, 60.38, -0.02, 101.41, -36.57]]
        
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        ik_req.robot_state = robot_state
        
        ik_req.avoid_collisions = True
        req.ik_request = ik_req
        
        future = self.ikin_client.call_async(req)
        # Wait for the future to finish without blocking the ROS 2 spin loop
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            
        res = future.result()
        # MoveIt GetPositionIK returns error_code.val == 1 for success
        if res and res.error_code.val == 1:
            joint_names = res.solution.joint_state.name
            joint_positions_rad = res.solution.joint_state.position
            joint_positions_deg = []
            for name in JOINT_NAMES:
                try:
                    idx = joint_names.index(name)
                    val_rad = joint_positions_rad[idx]
                    joint_positions_deg.append(np.degrees(val_rad))
                except ValueError:
                    self.get_logger().error(f"Joint {name} not found in IK solution!")
                    return None
            return joint_positions_deg
        else:
            err_val = res.error_code.val if res else "No response"
            self.get_logger().error(f"MoveIt IK solution failed for pose: {pose}. Error code: {err_val}")
            return None

    def get_current_pose_tf(self):
        """Gets current pose of tool0 relative to base_link using TF.
        Returns [x, y, z, qx, qy, qz, qw] where positions are in mm and orientation is quaternion."""
        try:
            now = rclpy.time.Time()
            # Wait up to 1.0s for the transform
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Position in mm
            x = trans.transform.translation.x * 1000.0
            y = trans.transform.translation.y * 1000.0
            z = trans.transform.translation.z * 1000.0
            
            # Orientation quaternion
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            return [x, y, z, qx, qy, qz, qw]
        except Exception as e:
            self.get_logger().error(f"Failed to lookup robot pose via TF: {e}")
            # Fallback to get_current_posx
            try:
                self.get_logger().info("Attempting fallback to get_current_posx...")
                posx, _ = get_current_posx()
                # Convert Euler ZYZ to quaternion
                r = Rotation.from_euler('ZYZ', [posx[3], posx[4], posx[5]], degrees=True)
                q = r.as_quat()
                return [posx[0], posx[1], posx[2], q[0], q[1], q[2], q[3]]
            except Exception as ex:
                self.get_logger().error(f"Fallback get_current_posx also failed: {ex}")
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    def move_to_joints_moveit(self, joint_positions_deg):
        """Plans and executes motion to the target joint positions (in degrees) using MoveGroup action.
        Aborts planning or execution if a hand is detected (unless in safety evasion mode)."""
        # Convert degrees to radians
        joint_positions_rad = [np.radians(angle) for angle in joint_positions_deg]
        
        # Build joint constraints
        goal_constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, joint_positions_rad):
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
        req.start_state.is_diff = True  # Start from current actual robot state
        req.goal_constraints = [goal_constraints]
        req.allowed_planning_time = 5.0
        # Velocity and acceleration scaling factors
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        opts = PlanningOptions()
        opts.plan_only = False  # Plan and execute immediately

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        # Check before sending goal (bypass check if in safety evasion mode)
        if self.hand_detected and not getattr(self, 'in_evasion', False):
            self.get_logger().warn('Hand detected before planning! Aborting motion.')
            return False

        self.get_logger().info('Sending MoveGroup Goal...')
        send_future = self._ac.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_future.done():
            if self.hand_detected and not getattr(self, 'in_evasion', False):
                self.get_logger().warn('Hand detected during planning phase! Aborting motion.')
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion planning request rejected by MoveGroup.')
            return False

        self.active_goal_handle = goal_handle
        self.get_logger().info('Motion planning accepted. Executing...')
        result_future = goal_handle.get_result_async()
        
        aborted = False
        while rclpy.ok() and not result_future.done():
            if self.hand_detected and not getattr(self, 'in_evasion', False):
                self.get_logger().warn('Hand detected during execution phase! Cancelling current motion...')
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    time.sleep(0.01)
                self.get_logger().info('Motion cancelled successfully.')
                aborted = True
                break
            time.sleep(0.05)

        self.active_goal_handle = None
        
        if aborted:
            return False

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Successfully moved to target joint positions!')
            return True
        else:
            self.get_logger().error(f'MoveGroup execution failed with error code: {result.error_code.val}')
            return False

    def execute_safety_evasion(self):
        """Safely retreats to home position (JReady) by avoiding the hand obstacle.
        If planning fails, it stops and waits until the path is clear."""
        JReady = [-44.26, 18.14, 60.38, -0.02, 101.41, -36.57]
        self.get_logger().warn("Safety Evasion triggered! Attempting to move to Home (JReady) by avoiding obstacles...")
        
        self.in_evasion = True
        while rclpy.ok():
            success = self.move_to_joints_moveit(JReady)
            if success:
                self.get_logger().info("Safety Evasion complete. Successfully returned to Home (JReady).")
                break
            else:
                self.get_logger().warn("Safety Evasion planning failed (hand might be directly blocking). Retrying in 2.0 seconds...")
                time.sleep(2.0)
                
        self.in_evasion = False

    def log_result_to_db(self, result, confidence):
        """Logs inspection result to the database."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO inspection_logs (result, confidence) VALUES (?, ?)",
            (result, float(confidence))
        )
        conn.commit()
        conn.close()
        self.get_logger().info(f"Logged result '{result}' with confidence {confidence:.2f} to database.")

    def get_robot_pose_matrix(self, x, y, z, qx, qy, qz, qw):
        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, yaw_cam, robot_pos=None):
        """Converts 3D coordinates and yaw angle from camera to base coordinates using TF."""
        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_color_optical_frame'
            point_cam.point.x = camera_coords[0] / 1000.0
            point_cam.point.y = camera_coords[1] / 1000.0
            point_cam.point.z = camera_coords[2] / 1000.0
            
            # Lookup transform from base_link to camera frame to get rotation
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Transform point to base frame
            point_base = self.tf_buffer.transform(point_cam, 'base_link')
            p_base = [point_base.point.x * 1000.0, point_base.point.y * 1000.0, point_base.point.z * 1000.0]
            
            # Extract rotation from TF to rotate the yaw vector
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            R_cam2base = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            
            # Construct direction vector of block axis in camera frame
            v_cam = np.array([np.cos(yaw_cam), np.sin(yaw_cam), 0.0])
            # Rotate direction vector to base frame
            v_base = R_cam2base @ v_cam
            # Project on table XY plane to compute base yaw
            yaw_base = np.arctan2(v_base[1], v_base[0])
            
            return p_base, yaw_base
            
        except Exception as e:
            self.get_logger().error(f"TF transform_to_base failed: {e}. Falling back to default calculations.")
            return [367.0, 3.0, 81.0], 0.0

    def get_spherical_pose(self, yaw_rad, pitch_deg, distance, target):
        """Computes camera target pose using spherical coordinate system."""
        d = distance
        pitch = np.radians(pitch_deg)
        r_xy = d * np.sin(pitch)
        z = target[2] + d * np.cos(pitch)
        
        c = np.array([
            target[0] + r_xy * np.cos(yaw_rad),
            target[1] + r_xy * np.sin(yaw_rad),
            z
        ])
        t = np.array(target)
        
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
        q = Rotation.from_matrix(R).as_quat()
        
        return c.tolist() + q.tolist()

    def get_tool_pose(self, cam_pose):
        """Converts desired camera pose [x,y,z,qx,qy,qz,qw] in base_link to tool0 pose using TF lookup."""
        T_b2c = np.eye(4)
        T_b2c[:3, 3] = cam_pose[:3]
        T_b2c[:3, :3] = Rotation.from_quat(cam_pose[3:]).as_matrix()
        
        try:
            # Lookup static transform from tool0 to camera_color_optical_frame
            # We use 0.0 seconds for lookup because it is a static transform
            trans = self.tf_buffer.lookup_transform('tool0', 'camera_color_optical_frame', rclpy.time.Time())
            t = trans.transform.translation
            r = trans.transform.rotation
            
            T_t2c = np.eye(4)
            T_t2c[:3, 3] = [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0]
            T_t2c[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            
            # T_base_to_tool0 = T_base_to_cam @ inv(T_tool0_to_cam)
            T_b2t = T_b2c @ np.linalg.inv(T_t2c)
            
            pos = T_b2t[:3, 3].tolist()
            q = Rotation.from_matrix(T_b2t[:3, :3]).as_quat().tolist()
            return pos + q
        except Exception as e:
            self.get_logger().error(f"Failed to transform camera pose to tool pose: {e}. Using camera pose directly.")
            return cam_pose

    def spawn_jenga_mesh(self, x, y, z, yaw):
        """Spawns standard STL mesh in RViz planning scene at computed coordinates."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'jenga_assembly'

        # Load Jenga STL mesh from m0609_rg2_bringup package
        mesh_dir = get_package_share_directory('m0609_rg2_bringup')
        mesh_path = os.path.join(mesh_dir, 'meshes', '이름없음-몸통.stl')

        if os.path.exists(mesh_path):
            try:
                tm = trimesh.load(mesh_path, force='mesh')
                mm_to_m = 0.001
                mesh = Mesh()
                for v in tm.vertices:
                    mesh.vertices.append(Point(x=float(v[0]) * mm_to_m, y=float(v[1]) * mm_to_m, z=float(v[2]) * mm_to_m))
                for f in tm.faces:
                    mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))
                obj.meshes = [mesh]
                
                pose = Pose()
                pose.position.x = x / 1000.0
                pose.position.y = y / 1000.0
                pose.position.z = z / 1000.0
                
                q = Rotation.from_euler('z', yaw).as_quat()
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                
                obj.mesh_poses = [pose]
                self.get_logger().info(f"Loaded mesh from {mesh_path} successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load mesh: {e}. Falling back to default Box.")
                self._apply_fallback_box(obj, x, y, z, yaw)
        else:
            self.get_logger().warn(f"Mesh file not found at {mesh_path}. Falling back to default Box.")
            self._apply_fallback_box(obj, x, y, z, yaw)

        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        self.scene_pub.publish(scene)
        self.get_logger().info("Planning scene update published.")

    def _apply_fallback_box(self, obj, x, y, z, yaw):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            JENGA_WIDTH / 1000.0,
            JENGA_DEPTH / 1000.0,
            JENGA_HEIGHT / 1000.0
        ]  # 75mm x 25mm x 15mm
        
        pose = Pose()
        pose.position.x = x / 1000.0
        pose.position.y = y / 1000.0
        pose.position.z = z / 1000.0
        
        q = Rotation.from_euler('z', yaw).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        obj.primitives = [box]
        obj.primitive_poses = [pose]

    def calculate_tcp_pose(self, camera_pos, target_pos, y_offset=0.0):
        """Spherical projection calculator (same as auto_dataset_capture_node)."""
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
        
        tcp_pos = c - y_offset * y_c
        q = Rotation.from_matrix(R).as_quat() # [x, y, z, w]
        return tcp_pos.tolist() + q.tolist()

    def handle_run_jenga_inspection(self, request, response):
        """Service callback that spawns the control loop in a separate thread to prevent deadlocks."""
        self.get_logger().info("Starting Jenga Inspection sequence in a separate thread...")
        thread = threading.Thread(target=self._run_inspection_thread, args=(request, response))
        thread.start()
        
        while rclpy.ok() and thread.is_alive():
            time.sleep(0.1)
            
        return response

    def _run_inspection_thread(self, request, response):
        """Background thread executing the Jenga inspection and scanning sequence."""
        JReady = [-44.26, 18.14, 60.38, -0.02, 101.41, -36.57]
        
        # Initialize/Reset states at start of inspection
        self.hand_detected = False
        self.current_hand_pos = None
        self.in_evasion = False
        self.clear_hand_obstacle()
        
        # Helper to ensure we move to JReady initially, with hand evasion check
        def move_to_initial_pose():
            while rclpy.ok():
                self.get_logger().info("Moving to initial scan pose JReady...")
                if self.move_to_joints_moveit(JReady):
                    return True
                if self.hand_detected:
                    self.get_logger().warn("Hand detected during initial pose movement. Evading to JReady...")
                    self.execute_safety_evasion()
                    self.get_logger().info("Waiting for hand to clear before resuming...")
                    while rclpy.ok() and self.hand_detected:
                        time.sleep(0.5)
                else:
                    # Generic motion planning failure (e.g. workspace limitation)
                    return False
            return False

        if not move_to_initial_pose():
            response.success = False
            response.message = "Failed to move to initial scan pose"
            return
                
        time.sleep(1.5) # Settle camera

        # Get real Jenga position via service call (ensure hand is cleared first)
        while rclpy.ok() and self.hand_detected:
            self.get_logger().warn("Hand detected. Waiting for hand to clear before camera scan...")
            time.sleep(1.0)

        self.get_logger().info("Calling /get_jenga_position service from side view...")
        if not self.get_position_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = "Perception node '/get_jenga_position' not available"
            return

        pos_req = SrvDepthPosition.Request()
        pos_req.target = "top"
        future = self.get_position_client.call_async(pos_req)
        
        # Wait for future to complete
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
            
        res = future.result()
        if not res or len(res.depth_position) < 4 or sum(res.depth_position[:3]) == 0:
            response.success = False
            response.message = "Failed to detect Jenga block 'top' or coordinates out of range"
            return
  
        x_cam, y_cam, z_cam, yaw_cam = res.depth_position[:4]
        self.get_logger().info(f"Detected Jenga camera pose: x={x_cam:.2f}, y={y_cam:.2f}, z={z_cam:.2f}, yaw={yaw_cam:.4f}")
  
        # Translate to Base frame using non-blocking TF lookup
        robot_posx = self.get_current_pose_tf()
        p_base, yaw_base = self.transform_to_base([x_cam, y_cam, z_cam], yaw_cam, robot_posx)
        self.get_logger().info(f"Translated Base pose: x={p_base[0]:.2f}, y={p_base[1]:.2f}, z={p_base[2]:.2f}, yaw={np.degrees(yaw_base):.2f}°")
  
        # 3. Spawn STL mesh in RViz
        spawn_z = p_base[2] - 42.0
        self.spawn_jenga_mesh(p_base[0], p_base[1], spawn_z, yaw_base)
  
        # 4. Generate 4 scanning viewpoints around the Jenga block using Spherical coordinates
        p_target = [p_base[0], p_base[1], p_base[2] - 42.0]
        
        face_names = ["Front", "Right", "Back", "Left"]
        successful_viewpoints = []
        
        for i in range(4):
            theta = yaw_base + i * (np.pi / 2.0)
            
            joint_angles = None
            for d in [280.0, 250.0, 310.0]:
                for p in [45.0, 35.0, 55.0]:
                    target_pose = self.get_spherical_pose(theta, p, d, p_target)
                    tool_pose = self.get_tool_pose(target_pose)
                    joint_angles = self.solve_ik(tool_pose)
                    if joint_angles:
                        break
                if joint_angles:
                    break
                    
            if joint_angles:
                successful_viewpoints.append((i, face_names[i], joint_angles))
            else:
                self.get_logger().warn(f"Could not find IK solution for Scan Position {i} ({face_names[i]}). Skipping.")

        # Choose adjacent faces
        scan_targets = []
        n_succ = len(successful_viewpoints)
        found_adjacent = False
        for i in range(n_succ):
            for j in range(i + 1, n_succ):
                idx1 = successful_viewpoints[i][0]
                idx2 = successful_viewpoints[j][0]
                diff = abs(idx1 - idx2)
                if diff == 1 or diff == 3:  # 90 deg rotation
                    scan_targets = [successful_viewpoints[i], successful_viewpoints[j]]
                    found_adjacent = True
                    break
            if found_adjacent:
                break
                
        if not found_adjacent and n_succ >= 2:
            scan_targets = successful_viewpoints[:2]
            
        self.get_logger().info(f"Selected {len(scan_targets)} viewpoints for scanning out of {len(successful_viewpoints)} successful configurations.")

        inspection_failed = False
        failed_reasons = []
        overall_confidence_list = []

        # Iterate targets with safety loop
        target_idx = 0
        while target_idx < len(scan_targets):
            face_idx, face_name, joint_angles = scan_targets[target_idx]
            self.get_logger().info(f"Moving to Scan Position {face_idx} ({face_name})...")
            
            # Try to move to the scan position
            success = self.move_to_joints_moveit(joint_angles)
            if not success:
                if self.hand_detected:
                    self.get_logger().warn(f"Hand detected while moving to Scan Position {face_name}! Initiating safety evasion...")
                    self.execute_safety_evasion()
                    
                    self.get_logger().warn("Waiting for hand to clear before resuming scan...")
                    while rclpy.ok() and self.hand_detected:
                        time.sleep(0.5)
                        
                    self.get_logger().info(f"Hand cleared. Resuming scan of {face_name} face (retrying same position)...")
                    # Do not increment target_idx; retry this same scan target
                    continue
                else:
                    self.get_logger().error(f"Failed to move to Scan Position {face_idx} via MoveGroup due to non-safety reasons. Skipping face.")
                    target_idx += 1
                    continue

            time.sleep(1.0) # Settle camera

            # Double check if hand was detected during settling time
            if self.hand_detected:
                self.get_logger().warn(f"Hand detected right after arrival at {face_name}! Initiating safety evasion...")
                self.execute_safety_evasion()
                self.get_logger().warn("Waiting for hand to clear before resuming scan...")
                while rclpy.ok() and self.hand_detected:
                    time.sleep(0.5)
                self.get_logger().info(f"Hand cleared. Resuming scan of {face_name} face...")
                continue # Retry this target position

            # Run YOLO feature detection
            if not self.detect_features_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Service /detect_jenga_features not available")
                target_idx += 1
                continue

            detect_req = Trigger.Request()
            detect_future = self.detect_features_client.call_async(detect_req)
            
            # Wait for perception to finish or check hand intrusion
            while rclpy.ok() and not detect_future.done():
                if self.hand_detected:
                    break
                time.sleep(0.1)
                
            if self.hand_detected:
                self.get_logger().warn(f"Hand detected during feature detection on {face_name}! Evading to JReady...")
                self.execute_safety_evasion()
                self.get_logger().warn("Waiting for hand to clear before resuming scan...")
                while rclpy.ok() and self.hand_detected:
                    time.sleep(0.5)
                self.get_logger().info(f"Hand cleared. Resuming scan of {face_name} face...")
                continue # Retry this target position

            detect_res = detect_future.result()
            if not detect_res or not detect_res.success:
                self.get_logger().warn(f"Failed to detect features on {face_name} face.")
                target_idx += 1
                continue

            # Parse JSON features
            try:
                features = json.loads(detect_res.message)
            except Exception as e:
                self.get_logger().error(f"Failed to parse JSON features: {e}")
                target_idx += 1
                continue

            # Count classes detected
            counts = {"smallhole": 0, "longhole": 0, "entire": 0}
            face_confidences = []
            for f in features:
                name = f["name"]
                if name in counts:
                    counts[name] += 1
                face_confidences.append(f["score"])

            avg_conf = np.mean(face_confidences) if face_confidences else 1.0
            overall_confidence_list.append(avg_conf)

            # Validate against EXPECTED_FEATURES
            expected = EXPECTED_FEATURES[face_idx]
            self.get_logger().info(f"[{face_name} Face] Detected: {counts} | Expected: {expected}")
            
            # Compare holes count
            if counts["smallhole"] != expected["smallhole"] or counts["longhole"] != expected["longhole"]:
                inspection_failed = True
                failed_reasons.append(
                    f"{face_name} face has wrong hole counts: "
                    f"small={counts['smallhole']} (expected {expected['smallhole']}), "
                    f"long={counts['longhole']} (expected {expected['longhole']})"
                )
            
            # Move on to next target
            target_idx += 1

        # 5. Determine overall inspection result and log to SQLite
        final_result = "FAIL" if inspection_failed else "PASS"
        avg_overall_confidence = np.mean(overall_confidence_list) if overall_confidence_list else 0.0
        self.log_result_to_db(final_result, avg_overall_confidence)

        # 6. Return back to Home position with safety evasion checks
        self.get_logger().info("Inspection complete. Returning to Home...")
        while rclpy.ok():
            if self.move_to_joints_moveit(JReady):
                break
            if self.hand_detected:
                self.execute_safety_evasion()
                while rclpy.ok() and self.hand_detected:
                    time.sleep(0.5)
            else:
                break

        # Cleanup obstacles
        self.clear_hand_obstacle()

        # Format output message
        if final_result == "PASS":
            response.success = True
            response.message = f"Jenga Inspection: PASS (Confidence: {avg_overall_confidence:.2f})"
        else:
            response.success = False
            response.message = f"Jenga Inspection: FAIL. Reasons: " + " | ".join(failed_reasons)

        return response


    def destroy_node(self):
        """Clean up the helper node and executor upon destruction."""
        self.action_executor.shutdown()
        self.action_node.destroy_node()
        super().destroy_node()


def main(args=None):
    node = JengaInspectorNode()
    
    # Use MultiThreadedExecutor so that blocking service callbacks do not deadlock action client callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    # Keep node alive in main thread
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == "__main__":
    main()
