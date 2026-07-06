#!/usr/bin/env python3
import os
import sys
# Force importing system OpenCV (4.5.4) with GUI support instead of local headless OpenCV (5.0.0)
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class HandTestNode(Node):
    def __init__(self):
        super().__init__('hand_test_node')
        
        # Load YOLOv8 hand detection model
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "resource", "hand_yolov8n.pt")
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            sys.exit(1)
            
        self.get_logger().info(f"Loading YOLOv8 hand model from: {model_path}")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.latest_image = None
        self.new_image_available = False
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to '/camera/camera/color/image_raw'")
        self.get_logger().info("Press 'ESC' or 'q' key in the video window to quit.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.new_image_available = True
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HandTestNode()
    
    import torch
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    node.get_logger().info(f"YOLO running on device: {device}")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            if node.latest_image is not None and node.new_image_available:
                node.new_image_available = False
                
                # Run inference
                results = node.model(node.latest_image, device=device, conf=0.5, verbose=False)
                annotated_frame = results[0].plot()
                
                # Show window
                cv2.imshow("YOLOv8 Hand Detection ROS 2 Test", annotated_frame)
                key = cv2.waitKey(20) & 0xFF
                if key == 27 or key == ord('q'): # ESC or q
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
