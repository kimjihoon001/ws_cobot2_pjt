#!/usr/bin/env python3
import os
import sys
import cv2
from ultralytics import YOLO

def main():
    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "resource", "hand_yolov8n.pt")
    
    if not os.path.exists(model_path):
        print(f"Error: Model not found at {model_path}")
        print("Please check if the package resources are set up correctly.")
        sys.exit(1)

    print("Loading YOLOv8 hand detection model...")
    model = YOLO(model_path)
    print("Model loaded successfully!")
    
    # Try different camera indices (default 0 for standard webcam or RealSense)
    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print("Invalid camera index argument. Using default 0.")

    print(f"Opening camera index {camera_index}...")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_index}.")
        print("Please make sure your camera is connected. You can try passing a different index, e.g.:")
        print(f"python3 {sys.argv[0]} 2  (or 4, etc.)")
        sys.exit(1)

    print("\n=== Real-time Hand Detection Test ===")
    print("Press 'q' key in the video window to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Exiting...")
            break
            
        # Run inference (use cuda if available)
        device = 'cuda' if YOLO().device.type == 'cuda' else 'cpu'
        # To avoid re-initializing YOLO device on every loop, we let ultralytics handle default device,
        # or we can explicitly pass device='cuda' if cuda is available.
        # Let's pass device='cpu' or device='cuda' depending on PyTorch configuration.
        import torch
        dev = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        results = model(frame, device=dev, conf=0.5, verbose=False)
        
        # Plot detection results on frame
        annotated_frame = results[0].plot()
        
        # Display the frame
        cv2.imshow("YOLOv8 Hand Detection Test", annotated_frame)
        
        # Wait for key press (10ms)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Test finished successfully.")

if __name__ == "__main__":
    main()
