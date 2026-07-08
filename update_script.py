import sys

with open('src/cobot2_ws/robot_control/robot_control/jenga_inspector.py', 'r') as f:
    content = f.read()

# 1. Update capture_single_frame
new_capture = """
    def capture_single_frame(self, face_name, pitch_deg):
        self.get_logger().info(f"[{face_name}] 각도 {pitch_deg}°에서 1프레임 캡처 중...")
        if self.latest_image is None or self.depth_frame is None or self.intrinsics is None:
            self.get_logger().warn("카메라 데이터를 기다리는 중...")
            time.sleep(0.5)
            return []
        
        frame_results = []
        img_copy = self.latest_image.copy()
        results = self.model(img_copy, conf=0.20, verbose=False)
        
        if len(results) > 0:
            res = results[0]
            annotated_frame = res.plot()
            
            # 이미지 저장 (백엔드 static 폴더)
            possible_paths = [
                os.path.expanduser("~/ws_cobot2_pjt/backend"),
                os.path.expanduser("~/cobot_ws/src/ws_cobot2_pjt/backend")
            ]
            db_dir = possible_paths[0]
            for p in possible_paths:
                if os.path.exists(p):
                    db_dir = p
                    break
            
            save_dir = os.path.join(db_dir, "..", "static", "inspection_images")
            os.makedirs(save_dir, exist_ok=True)
            timestamp = int(time.time() * 1000)
            img_filename = f"inspection_{face_name}_{pitch_deg}_{timestamp}.jpg"
            filename = os.path.join(save_dir, img_filename)
            cv2.imwrite(filename, annotated_frame)
            
            if abs(pitch_deg - 45.0) < 1.0:
                if not hasattr(self, 'current_inspection_images'):
                    self.current_inspection_images = []
                self.current_inspection_images.append(f"/static/inspection_images/{img_filename}")
            
            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(img_msg)
            except Exception:
                pass
            
            names = res.names
            entire_box = None
            hole_boxes = []
            
            for box in res.boxes:
"""

import re
content = re.sub(
    r'    def capture_single_frame\(self, face_name, pitch_deg\):.*?for box in res\.boxes:',
    new_capture.strip(),
    content,
    flags=re.DOTALL
)

# 2. Update log_result_to_db signature and map_data
old_log = """    def log_result_to_db(self, product, result, defect_location, map_data_json):"""
new_log = """    def log_result_to_db(self, product, result, defect_location, map_data_json, image_paths=None):
        import json
        if image_paths and len(image_paths) > 0:
            try:
                map_dict = json.loads(map_data_json)
                map_dict["images"] = image_paths
                map_data_json = json.dumps(map_dict)
            except Exception:
                pass"""

content = content.replace(old_log, new_log)

# 3. Update the call to log_result_to_db in _run_inspection_thread
old_call = """            self.log_result_to_db(product_name_for_db, final_result, defect_loc, map_data_json)"""
new_call = """            imgs = getattr(self, 'current_inspection_images', [])
            self.log_result_to_db(product_name_for_db, final_result, defect_loc, map_data_json, imgs)
            self.current_inspection_images = []  # reset for next inspection"""

content = content.replace(old_call, new_call)

with open('src/cobot2_ws/robot_control/robot_control/jenga_inspector.py', 'w') as f:
    f.write(content)
print("Updated jenga_inspector.py successfully")
