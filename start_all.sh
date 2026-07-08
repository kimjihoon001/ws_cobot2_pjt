#!/bin/bash

# ==============================================================================
# 실물 로봇 통합 실행
# - 음성 젠가 품질검사
# - 음성 공구 픽업/배송
# 공통 ROS/카메라/HMI 노드는 한 번만 실행하고 두 작업 노드는 함께 대기시킨다.
# ==============================================================================

WS_DIR="$HOME/ws_cobot2_pjt"
if [ ! -d "$WS_DIR" ]; then
  WS_DIR="$HOME/cobot_ws/src/ws_cobot2_pjt"
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "❌ ROS 2 워크스페이스를 찾을 수 없습니다: $WS_DIR"
  echo "먼저 colcon build --symlink-install을 실행해 주세요."
  exit 1
fi

if ! command -v terminator >/dev/null 2>&1; then
  echo "❌ terminator가 설치되어 있지 않습니다."
  exit 1
fi

ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
LAYOUT_GENERATOR="$WS_DIR/scripts/gen_terminator_layout.py"
PICK_SCRIPT="$WS_DIR/src/cobot2_ws/m0609_rg2_bringup/scripts/tool_pick_yolo_target.py"
ENV_FILE="$WS_DIR/src/cobot2_ws/voice_processing/resource/.env"
LAYOUT_FILE="/tmp/cobot_all_terminator_layout.conf"
TERMINATOR_LOG="/tmp/cobot_all_terminator.log"
ROS_ENV="export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash"

for required_file in "$LAYOUT_GENERATOR" "$PICK_SCRIPT" "$ENV_FILE"; do
  if [ ! -f "$required_file" ]; then
    echo "❌ 필수 파일을 찾을 수 없습니다: $required_file"
    exit 1
  fi
done

wrap() { echo "bash -lc 'echo \"$1\"; $2; exec bash'"; }

python3 "$LAYOUT_GENERATOR" "$LAYOUT_FILE" \
  "0-RealSense"      "$(wrap 'RealSense 카메라 구동 중...' "$ROS_ENV && ros2 launch realsense2_camera rs_align_depth_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=1280x720x30 initial_reset:=true align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true")" \
  "1-Bringup"        "$(wrap '실물 로봇 드라이버 기동 중...' "$ROS_ENV && ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=$ROBOT_HOST")" \
  "2-MoveIt"         "$(wrap 'MoveIt 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 launch m0609_rg2_moveit movegroup_only.launch.py")" \
  "3-Jenga-YOLO"     "$(wrap '젠가 검출 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run object_detection jenga_detection")" \
  "4-Hand-YOLO"      "$(wrap 'YOLO 손 검출 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run object_hand object_hand")" \
  "5-Hand-Obstacle"  "$(wrap '손 장애물 퍼블리셔 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run object_hand hand_obstacle_publisher")" \
  "6-Jenga-Inspector" "$(wrap '젠가 검사 노드 대기 중 (8초)...' "sleep 8; $ROS_ENV && ros2 run robot_control jenga_inspector")" \
  "7-Backend"        "$(wrap 'HMI 백엔드와 ROS 브릿지 구동 중...' "export PYTHONUNBUFFERED=1; $ROS_ENV && cd $WS_DIR/backend && python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000")" \
  "8-Frontend"       "$(wrap 'HMI 프론트엔드 구동 중...' "cd $WS_DIR/frontend && npm run dev -- --host 0.0.0.0 --port 5173")" \
  "9-get_keyword"    "$(wrap '음성 키워드 노드 대기 중 (8초)...' "sleep 8; $ROS_ENV && ros2 run voice_processing get_keyword")" \
  "10-Tool-Pick"     "$(wrap '공구 픽업 노드 대기 중 (10초)...' "sleep 10; $ROS_ENV && python3 $PICK_SCRIPT")" \
  "11-Monitor"       "$(wrap 'ROS 상태 확인 터미널' "$ROS_ENV")"

echo "🚀 통합 시스템을 Terminator 12분할로 실행합니다..."
terminator --no-dbus --maximise -g "$LAYOUT_FILE" -l main >"$TERMINATOR_LOG" 2>&1 &
TERMINATOR_PID=$!

sleep 1
if ! kill -0 "$TERMINATOR_PID" 2>/dev/null; then
  echo "❌ Terminator 실행에 실패했습니다."
  cat "$TERMINATOR_LOG"
  exit 1
fi

disown "$TERMINATOR_PID" 2>/dev/null || true

echo "✅ 실물 통합 시스템 실행 완료"
echo "로봇: $ROBOT_HOST, ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "HMI: http://localhost:5173"
echo "명령: '품질검사해' 또는 '해머/드라이버 가져다줘'"
echo "Terminator 로그: $TERMINATOR_LOG"
