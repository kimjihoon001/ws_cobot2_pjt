#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from od_msg.srv import SrvDepthPosition
import tf2_ros
import tf2_geometry_msgs  # Crucial for transforming PointStamped


class HandObstaclePublisher(Node):
    def __init__(self):
        super().__init__('hand_obstacle_publisher')

        # 1. Parameter declaration
        self.declare_parameter('target_tool', 'hand')  # Set default target to hand
        self.declare_parameter('update_rate', 2.0)  # Rate in Hz to request coordinates
        self.declare_parameter('clear_timeout', 4.0)  # Seconds to persist obstacle after losing it

        self.target_tool = self.get_parameter('target_tool').get_parameter_value().string_value
        rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.clear_timeout = self.get_parameter('clear_timeout').get_parameter_value().double_value

        # Memory variables to persist obstacle when camera rotates away
        self.last_detection_time = None
        self.last_known_pos = None

        # 2. Service Client to query hand 3D position
        self.client = self.create_client(SrvDepthPosition, 'get_hand_position')
        self.get_logger().info(f"Waiting for 'get_hand_position' service...")
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service 'get_hand_position' not available, waiting...")

        # 3. TF Listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. Publisher for the obstacle position (MoveIt hand avoidance node subscribes here)
        self.obstacle_pub = self.create_publisher(Point, '/hand_position', 10)

        # 5. Timer to request position periodically
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"HandObstaclePublisher initialized. Target: {self.target_tool} at {rate} Hz (Persistence: {self.clear_timeout}s).")
        self.is_busy = False

    def timer_callback(self):
        if self.is_busy:
            return

        self.is_busy = True
        req = SrvDepthPosition.Request()
        req.target = self.target_tool

        # Call service asynchronously
        future = self.client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            pos = response.depth_position
            
            # Check if detection was successful (a valid 3D point is not [0, 0, 0])
            if len(pos) >= 3 and (pos[0] != 0.0 or pos[1] != 0.0 or pos[2] != 0.0):
                # The service returns coords in millimeters.
                # TF and MoveIt require coordinates in METERS.
                x_m = pos[0] / 1000.0
                y_m = pos[1] / 1000.0
                z_m = pos[2] / 1000.0

                self.get_logger().info(f"Detected {self.target_tool} in camera frame: x={x_m:.3f}m, y={y_m:.3f}m, z={z_m:.3f}m")

                # Create PointStamped in the camera color optical frame
                pt_cam = PointStamped()
                pt_cam.header.frame_id = 'camera_color_optical_frame'
                pt_cam.header.stamp = self.get_clock().now().to_msg()
                pt_cam.point.x = x_m
                pt_cam.point.y = y_m
                pt_cam.point.z = z_m

                # Look up the transform from camera optical frame to robot base_link
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',
                        'camera_color_optical_frame',
                        rclpy.time.Time()
                    )
                    
                    # Transform the point
                    pt_base = tf2_geometry_msgs.do_transform_point(pt_cam, transform)

                    # Publish to /hand_position
                    out_msg = Point()
                    out_msg.x = pt_base.point.x
                    out_msg.y = pt_base.point.y
                    out_msg.z = pt_base.point.z
                    
                    self.obstacle_pub.publish(out_msg)
                    self.get_logger().info(f"Published obstacle in base_link: x={out_msg.x:.3f}m, y={out_msg.y:.3f}m, z={out_msg.z:.3f}m")

                    # Update persistence memory
                    self.last_detection_time = self.get_clock().now()
                    self.last_known_pos = out_msg

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f"Failed to transform tool coordinate to base_link: {e}")
            else:
                # No detection in the current frame. Check if we should hold the last position.
                if self.last_detection_time is not None:
                    elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    if elapsed < self.clear_timeout:
                        if self.last_known_pos is not None:
                            self.obstacle_pub.publish(self.last_known_pos)
                            self.get_logger().info(f"Target lost (FOV limit). Persisting last known position. (Expires in {self.clear_timeout - elapsed:.1f}s)")
                            return

                # Reached timeout or no previous detection: Clear from scene
                clear_msg = Point()
                clear_msg.x = 0.0
                clear_msg.y = 0.0
                clear_msg.z = -2.0
                self.obstacle_pub.publish(clear_msg)
                
                if self.last_detection_time is not None:
                    self.get_logger().info("Persistence timeout elapsed. Clearing obstacle from planning scene.")
                    self.last_detection_time = None
                    self.last_known_pos = None

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            self.is_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = HandObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
