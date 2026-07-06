import os
import time
import json
from collections import Counter

import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from object_detection.realsense import ImgNode

PACKAGE_NAME = "object_detection"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "bestjenga.onnx"
YOLO_CLASS_NAME_JSON = "class_name_jenga.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)


class JengaYoloModel:
    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # Load Jenga ONNX model
        self.model = YOLO(YOLO_MODEL_PATH, task='detect')
        
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            # e.g., {"0": "entire", "1": "longhole", "2": "smallhole"}
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}
            self.class_names = {int(k): v for k, v in class_dict.items()}

        print(f"Jenga YOLO model initialized on device: {self.device}")
        print(f"Classes: {self.class_names}")

    def get_detections(self, frame, confidence_threshold=0.5):
        """Runs inference and returns all detections above confidence threshold."""
        results = self.model(frame, device=self.device, verbose=False)
        detections = []
        if len(results) > 0:
            res = results[0]
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    detections.append({
                        "box": box,
                        "score": score,
                        "label": int(label),
                        "name": self.class_names.get(int(label), "unknown")
                    })
        return detections


class JengaDetectionNode(Node):
    def __init__(self):
        super().__init__('jenga_detection_node')
        self.img_node = ImgNode()
        self.model = JengaYoloModel()
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        
        # Create services
        self.create_service(
            SrvDepthPosition,
            'get_jenga_position',
            self.handle_get_jenga_position
        )
        self.create_service(
            Trigger,
            'detect_jenga_features',
            self.handle_detect_jenga_features
        )
        self.get_logger().info("JengaDetectionNode initialized and ready.")

    def handle_detect_jenga_features(self, request, response):
        self.get_logger().info("Received request for Jenga feature detection.")
        for _ in range(5):
            rclpy.spin_once(self.img_node, timeout_sec=0.001)

        color_frame = self.img_node.get_color_frame()
        if color_frame is None:
            response.success = False
            response.message = "Color frame not available"
            return response

        detections = self.model.get_detections(color_frame, confidence_threshold=0.5)
        result_list = []
        
        # Draw bounding boxes on copy of the frame to save
        annotated_frame = color_frame.copy()
        
        for d in detections:
            name = d["name"]
            box = d["box"]
            score = d["score"]
            result_list.append({
                "name": name,
                "box": [float(x) for x in box],
                "score": float(score)
            })
            
            # Draw boxes: entire (green), longhole (blue), smallhole (red)
            x1, y1, x2, y2 = map(int, box)
            if name == "entire":
                color = (0, 255, 0)
            elif name == "longhole":
                color = (255, 0, 0)
            else:
                color = (0, 0, 255)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated_frame, f"{name}: {score:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Save annotated image to local folder ~/inspection_images/
        save_dir = os.path.join(os.path.expanduser('~'), 'inspection_images')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            self.get_logger().info(f"Created directory for saving images: {save_dir}")
        
        timestamp = int(time.time() * 1000)
        filename = os.path.join(save_dir, f"inspection_{timestamp}.jpg")
        cv2.imwrite(filename, annotated_frame)
        self.get_logger().info(f"Saved annotated inspection image to {filename}")

        response.success = True
        response.message = json.dumps(result_list)
        return response

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retrying to get {description}...")
            time.sleep(0.1)
            data = getter()
        return data

    def handle_get_jenga_position(self, request, response):
        self.get_logger().info(f"Received request for target: {request.target}")
        coords = self._compute_position_and_yaw(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position_and_yaw(self, target):
        # Spin to refresh frame
        for _ in range(5):
            rclpy.spin_once(self.img_node, timeout_sec=0.001)

        color_frame = self.img_node.get_color_frame()
        depth_frame = self.img_node.get_depth_frame()

        if color_frame is None or depth_frame is None:
            self.get_logger().warn("Color or depth frame is not available.")
            return 0.0, 0.0, 0.0, 0.0

        # Run YOLO inference
        detections = self.model.get_detections(color_frame, confidence_threshold=0.5)
        
        # Filter for the target (e.g. 'entire' or checking other features)
        target_detections = [d for d in detections if d["name"] == target]
        if not target_detections:
            self.get_logger().warn(f"Target '{target}' not detected.")
            return 0.0, 0.0, 0.0, 0.0

        # Choose the detection with the highest confidence
        best_det = max(target_detections, key=lambda x: x["score"])
        box = best_det["box"]
        self.get_logger().info(f"Best detection: {best_det}")

        x1, y1, x2, y2 = map(int, box)
        h, w = depth_frame.shape

        # Make sure bounding box is within frame limits
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w - 1, x2)
        y2 = min(h - 1, y2)

        # Get depth slice for the bounding box
        depth_crop = depth_frame[y1:y2, x1:x2]
        
        # Use median of non-zero depth values in the box as the representative depth
        valid_depths = depth_crop[depth_crop > 0]
        if len(valid_depths) == 0:
            self.get_logger().warn("No valid depth information in the bounding box.")
            return 0.0, 0.0, 0.0, 0.0

        cz = float(np.median(valid_depths))
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Project 2D center to 3D camera coordinates
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']

        x_cam = (cx - ppx) * cz / fx
        y_cam = (cy - ppy) * cz / fy
        z_cam = cz

        # Estimate Yaw angle using OpenCV minAreaRect on depth segmentation mask
        # Segment pixels that belong to the Jenga block (within cz +/- 20 mm)
        depth_threshold = 20.0  # mm
        mask = ((depth_crop > cz - depth_threshold) & (depth_crop < cz + depth_threshold)).astype(np.uint8) * 255

        # Perform morphological cleaning
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yaw_rad = 0.0

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 50:
                rect = cv2.minAreaRect(largest_contour)
                (cx_rect, cy_rect), (w_rect, h_rect), angle = rect

                # Standardize angle to represent the primary orientation of the rectangle
                if w_rect < h_rect:
                    angle_deg = angle + 90.0
                else:
                    angle_deg = angle

                # Convert to radians relative to camera X-axis
                yaw_rad = np.radians(angle_deg)
                self.get_logger().info(f"Yaw estimated: {angle_deg:.2f} deg ({yaw_rad:.4f} rad)")
            else:
                self.get_logger().info("Segmented contour is too small. Defaulting yaw to 0.0.")
        else:
            self.get_logger().info("No contours found in the depth mask. Defaulting yaw to 0.0.")

        # Return camera 3D coordinates (x, y, z in mm) and yaw angle in camera frame
        return x_cam, y_cam, z_cam, yaw_rad


def main(args=None):
    rclpy.init(args=args)
    node = JengaDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
