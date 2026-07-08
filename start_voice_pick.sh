#!/bin/bash

# ==============================================================================
# 음성 명령 -> 도구 픽업/배송 시스템 전체 실행 스크립트
# terminator 창 하나를 10분할해서 전부 띄운다 (RealSense + 실제 손 검출 +
# 빈 터미널 1개 포함). ~/.config/terminator/config는 안 건드리고,
# scripts/gen_terminator_layout.py로 별도 레이아웃 파일을 만들어서 그걸로만 띄운다.
# (start_jenga.sh / terminator_jenga.config 규칙과 동일한 경로/방식으로 맞춤)
#
# 두산 m0609 실물 로봇(기본 IP: 192.168.1.100) 전용 실행 스크립트다.
# ==============================================================================

# 워크스페이스 경로 자동 감지 (start_jenga.sh와 동일한 규칙)
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

SCRIPTS_DIR="$WS_DIR/src/cobot2_ws/m0609_rg2_bringup/scripts"
LAYOUT_GENERATOR="$WS_DIR/scripts/gen_terminator_layout.py"
PICK_SCRIPT="$SCRIPTS_DIR/tool_pick_yolo_target.py"
ROS_ENV="export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash"
LAYOUT_FILE="/tmp/voice_pick_terminator_layout.conf"
TERMINATOR_LOG="/tmp/voice_pick_terminator.log"

for required_file in "$LAYOUT_GENERATOR" "$PICK_SCRIPT"; do
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
  "3-YOLO-Hand"      "$(wrap 'YOLO 손 검출 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run object_hand object_hand")" \
  "4-HandObstacle"   "$(wrap '손 장애물 퍼블리셔 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run object_hand hand_obstacle_publisher")" \
  "5-Backend"        "$(wrap 'HMI 백엔드(ROS2 브릿지 포함) 구동 중... (--reload 금지)' "export PYTHONUNBUFFERED=1; $ROS_ENV && cd $WS_DIR/backend && python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000")" \
  "6-Frontend"       "$(wrap 'HMI 프론트엔드 구동 중...' "cd $WS_DIR/frontend && npm run dev -- --host 0.0.0.0 --port 5173")" \
  "7-get_keyword"    "$(wrap '음성 인식 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run voice_processing get_keyword")" \
  "8-tool_pick_yolo" "$(wrap '도구 픽업 스크립트 대기 중 (8초)...' "sleep 8; $ROS_ENV && python3 $PICK_SCRIPT")" \
  "9-blank"          ""

echo "🚀 terminator 창 하나에 10분할로 엽니다..."
terminator --no-dbus --maximise -g "$LAYOUT_FILE" -l main >"$TERMINATOR_LOG" 2>&1 &
TERMINATOR_PID=$!

sleep 1
if ! kill -0 "$TERMINATOR_PID" 2>/dev/null; then
  echo "❌ Terminator 실행에 실패했습니다."
  cat "$TERMINATOR_LOG"
  exit 1
fi
disown "$TERMINATOR_PID" 2>/dev/null || true

echo "✅ 실물 로봇 모드로 실행됐습니다! host: $ROBOT_HOST, ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "HMI: http://localhost:5173 -> 로그인 -> 홈에서 음성 시작 버튼"
echo "Terminator 로그: $TERMINATOR_LOG"
