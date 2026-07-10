import os
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import torch
import cv2
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

from od_msg.srv import YoloInference
from object_detection.realsense import ImgNode

# Define model paths based on the previous scripts
PKG_OBJ_DETECTION = get_package_share_directory('object_detection')
PKG_OBJ_HAND = get_package_share_directory('object_hand')

# Jenga detection
JENGA_MODEL_PATH = os.path.join(PKG_OBJ_DETECTION, 'resource', 'bestjenga.onnx')
JENGA_JSON_PATH = os.path.join(PKG_OBJ_DETECTION, 'resource', 'class_name_jenga.json')

# Hand detection
HAND_MODEL_PATH = os.path.join(PKG_OBJ_HAND, 'resource', 'hand_yolov8n.pt')

# Tool pick
TOOL_MODEL_PATH = os.path.expanduser('~/ws_cobot2_pjt/src/cobot2_ws/voice_processing/resource/toolbest.pt')
if not os.path.exists(TOOL_MODEL_PATH):
    TOOL_MODEL_PATH = os.path.expanduser('~/cobot_ws/src/ws_cobot2_pjt/src/cobot2_ws/voice_processing/resource/toolbest.pt')

# Jenga inspection
WORKSPACE_PATH = os.path.expanduser('~/ws_cobot2_pjt')
if not os.path.exists(WORKSPACE_PATH):
    WORKSPACE_PATH = os.path.expanduser('~/cobot_ws/src/ws_cobot2_pjt')

INSPECTOR_MODEL_PATH = os.path.join(WORKSPACE_PATH, 'src/yolov8_ws/model/best_3.onnx')
if not os.path.exists(INSPECTOR_MODEL_PATH):
    INSPECTOR_MODEL_PATH = os.path.join(WORKSPACE_PATH, 'src/yolov8_ws/model/best_2.onnx')

class UnifiedVisionServer(Node):
    def __init__(self):
        super().__init__('unified_vision_server')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Initializing Unified Vision Server on {self.device}")
        
        # We share ONE camera node
        self.img_node = ImgNode()

        # Load all models once into the same process
        self.models = {}
        
        self.get_logger().info("Loading jenga_detection model...")
        self.models['jenga'] = YOLO(JENGA_MODEL_PATH, task='detect')
        with open(JENGA_JSON_PATH, "r", encoding="utf-8") as f:
            class_dict = json.load(f)
            self.jenga_class_names = {int(k): v for k, v in class_dict.items()}

        self.get_logger().info("Loading hand model...")
        self.models['hand'] = YOLO(HAND_MODEL_PATH)
        
        self.get_logger().info("Loading tool pick model...")
        self.models['tool'] = YOLO(TOOL_MODEL_PATH)
        
        self.get_logger().info("Loading jenga inspection model...")
        self.models['inspector'] = YOLO(INSPECTOR_MODEL_PATH, task='detect')

        # Create a reentrant callback group for multi-threading
        self.callback_group = ReentrantCallbackGroup()

        # Create the inference service
        self.srv = self.create_service(
            YoloInference,
            '/vision/get_bboxes',
            self.handle_get_bboxes,
            callback_group=self.callback_group
        )
        self.get_logger().info("Unified Vision Server is ready! (Multi-Threaded)")

    def handle_get_bboxes(self, request, response):
        model_name = request.model_name
        conf_thresh = request.confidence_threshold if request.confidence_threshold > 0 else 0.5
        
        if model_name not in self.models:
            response.success = False
            response.json_result = json.dumps({"error": f"Model {model_name} not found"})
            return response
            
        # Spin to get latest frame is no longer needed because img_node is spun by the executor
        frame = self.img_node.get_color_frame()
        if frame is None:
            response.success = False
            response.json_result = json.dumps({"error": "No color frame available"})
            return response

        # Run inference
        model = self.models[model_name]
        results = model(frame, device=self.device, verbose=False)
        
        detections = []
        if len(results) > 0:
            res = results[0]
            for i, (box, score, label) in enumerate(zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist(), res.boxes.cls.tolist())):
                if score >= conf_thresh:
                    cls_id = int(label)
                    
                    # Resolve class names
                    name = str(cls_id)
                    if model_name == 'jenga':
                        name = self.jenga_class_names.get(cls_id, name)
                    elif hasattr(model, 'names') and isinstance(model.names, dict):
                        name = model.names.get(cls_id, name)
                        
                    det = {
                        "name": name,
                        "class_id": cls_id,
                        "box": [float(x) for x in box],
                        "score": float(score)
                    }
                    if hasattr(res, 'keypoints') and res.keypoints is not None and len(res.keypoints) > 0:
                        try:
                            kpts = res.keypoints.xy[i].tolist()
                            det["keypoints"] = kpts
                        except Exception:
                            pass
                    detections.append(det)
                    
        response.success = True
        response.json_result = json.dumps(detections)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedVisionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.img_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
