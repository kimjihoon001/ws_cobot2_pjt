# Project Specifications and Rules

This project is a Collaborative Robot Smart Assembly and Quality Inspection System using Speech Control, YOLO, and Depth Vision.

## System Hardware
- **Robot Arm**: Doosan Robotics m0609 (두산 m0609)
- **Gripper**: OnRobot RG2
- **Camera**: Intel RealSense D435Y

## Software Stack
- **OS/Middleware**: Ubuntu 22.04 LTS (Jammy) + ROS 2 Humble
- **Motion Planning**: MoveIt 2 (Python/C++ APIs)
- **Object Detection & Quality Inspection**: YOLOv8 (OpenCV / PyRealSense2 / realsense2_camera)
- **Database**: SQLite 3 (inventory tracking and inspection logs)
- **HRI**: Python SpeechRecognition (Google STT)
- **User Interface**: Streamlit Dashboard (for real-time inventory and statistics)

## Project-Scoped Rules
- Always use ROS 2 Humble commands and standards.
- When writing robot control nodes, refer to the Doosan m0609 interfaces.
- For motion planning, use Joint Space Control as the default target mode to avoid singularity/out-of-reach issues.
- Set a virtual boundary/collision box (Safety Fence) in the MoveIt planning scene to protect the human operator.
- Always write proposals and implementation plans in Korean. (제안서 및 구현 계획서는 항상 한국어로 작성합니다.)
- Always write Git commit messages in Korean. (Git 커밋 메시지는 항상 한국어로 작성합니다.)

