#!/bin/bash

# ==============================================================================
# 젠가 불량 검사 및 HMI 통합 시스템 자동 실행 스크립트 (gnome-terminal 탭 모드)
# ==============================================================================

echo "🚀 젠가 불량 검사 시스템을 시작합니다... 여러 개의 터미널 탭이 한 번에 열립니다."

# 워크스페이스 경로 자동 감지
WS_DIR="$HOME/ws_cobot2_pjt"
if [ ! -d "$WS_DIR" ]; then
    WS_DIR="$HOME/cobot_ws/src/ws_cobot2_pjt"
fi

gnome-terminal --window --maximize \
  --tab --title="[1] RealSense" -- bash -c "echo 'RealSense 카메라 구동 중...'; realsense; exec bash" \
  --tab --title="[2] Robot Driver" -- bash -c "echo '로봇 드라이버 기동 중...'; cd $WS_DIR && source install/setup.bash && ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100; exec bash" \
  --tab --title="[3] MoveIt" -- bash -c "echo 'MoveIt 대기 중 (5초)...'; sleep 5; cd $WS_DIR && source install/setup.bash && ros2 launch m0609_rg2_moveit movegroup_only.launch.py; exec bash" \
  --tab --title="[4] YOLO Jenga" -- bash -c "echo 'YOLO 젠가 검출 노드 구동 중...'; cd $WS_DIR && source install/setup.bash && ros2 run object_detection jenga_detection; exec bash" \
  --tab --title="[5] YOLO Hand" -- bash -c "echo 'YOLO 손 검출 노드 구동 중...'; cd $WS_DIR && source install/setup.bash && ros2 run object_hand object_hand; exec bash" \
  --tab --title="[6] Hand Obstacle" -- bash -c "echo '손 회피 장애물 퍼블리셔 구동 중...'; cd $WS_DIR && source install/setup.bash && ros2 run object_hand hand_obstacle_publisher; exec bash" \
  --tab --title="[7] Inspector (Main)" -- bash -c "echo '젠가 검사 메인 제어 노드 구동 중...'; cd $WS_DIR && source install/setup.bash && ros2 run robot_control jenga_inspector; exec bash" \
  --tab --title="[8] ROS Bridge" -- bash -c "echo '웹소켓 브릿지 구동 중...'; cd $WS_DIR && source install/setup.bash && ros2 run rosbridge_server rosbridge_websocket; exec bash" \
  --tab --title="[9] FastAPI Backend" -- bash -c "echo '웹 백엔드 구동 중...'; cd $WS_DIR/backend && uvicorn app.main:app --reload; exec bash" \
  --tab --title="[10] Vite Frontend" -- bash -c "echo '웹 프론트엔드 구동 중...'; cd $WS_DIR/frontend && npm run dev; exec bash"

echo "✅ 모든 터미널 탭이 실행되었습니다! 새로 뜬 터미널 창을 확인해 주세요."
