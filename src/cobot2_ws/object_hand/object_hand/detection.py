import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_hand.realsense import ImgNode
from object_hand.yolo import YoloModel


PACKAGE_NAME = 'object_hand'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectHandNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_hand_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_hand_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectHandNode initialized.")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)

        # Force target search to 'hand'
        box, score = self.model.get_best_detection(self.img_node, 'hand')
        if box is None or score is None:
            self.get_logger().warn("No hand detection found.")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표 주변의 유효한 최소 depth 값을 읽어옵니다 (배경 무시, 근접 무효영역 및 노이즈 필터링)."""
        # YOLO 연산 등으로 지연된 동안 새로 들어온 최신 depth 메시지를 플러시
        for _ in range(5):
            rclpy.spin_once(self.img_node, timeout_sec=0.001)

        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        if frame is None:
            return None
        
        try:
            h, w = frame.shape
            # 15x15 윈도우 설정 (배경 대비 객체 판단을 용이하게 하기 위해 약간 확장)
            r = 7
            x_min = max(0, x - r)
            x_max = min(w - 1, x + r)
            y_min = max(0, y - r)
            y_max = min(h - 1, y + r)
            
            window = frame[y_min:y_max+1, x_min:x_max+1]
            total_pixels = window.size
            zero_pixels = np.sum(window == 0)
            zero_ratio = zero_pixels / total_pixels
            
            # RealSense D435의 최소 측정 거리(약 15cm)보다 너무 가까워져서 depth가 0(무효)으로 깨지는 현상 감지
            # 배경 테이블은 깊이가 잘 측정되므로(0이 거의 없음), 윈도우 내 무효 영역 비율이 높으면 공구가 매우 가까이 들어온 것으로 판단.
            if zero_ratio > 0.25:
                self.get_logger().info(f"[Heuristic] High invalid depth ratio ({zero_ratio:.2f}) detected. Object is likely too close (in the air). Returning 150mm.")
                return 150.0
            
            valid_depths = window[window > 0]
            good_depths = valid_depths[(valid_depths >= 150) & (valid_depths <= 1500)]
            
            if len(good_depths) > 0:
                min_depth = np.min(good_depths)
                # 만약 가장 가까운 깊이가 34cm 미만이면 확실히 공중에 떠 있는 상태
                if min_depth < 340.0:
                    return float(min_depth)
                
                # 그 외에는 테이블 바닥 깊이 반환 (보통 380~410mm)
                return float(min_depth)
            
            # 필터링된 깊이가 없으면 센터 픽셀 직접 시도
            val = frame[y, x]
            return float(val) if val > 0 else None
            
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None
        except Exception as e:
            self.get_logger().warn(f"Failed to get depth: {e}")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
