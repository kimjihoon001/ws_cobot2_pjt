#!/bin/bash

# ==============================================================================
# 음성 명령 -> 도구 픽업/배송 시스템 전체 실행 스크립트
# terminator 창 하나를 10분할해서 전부 띄운다 (realsense 별칭 + hand_avoidance +
# 빈 터미널 1개 포함). ~/.config/terminator/config는 안 건드리고,
# scripts/gen_terminator_layout.py로 별도 레이아웃 파일을 만들어서 그걸로만 띄운다.
# (start_jenga.sh / terminator_jenga.config 규칙과 동일한 경로/방식으로 맞춤)
#
# 가상 모드(Docker DRCF 에뮬레이터) 기준. 실물 로봇은 [Bringup] 항목의
# bringup_camera.launch.py 뒤에 mode:=real host:=<로봇IP> 를 추가할 것.
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

SCRIPTS_DIR="$WS_DIR/src/cobot2_ws/m0609_rg2_bringup/scripts"
ROS_ENV="export ROS_DOMAIN_ID=70 && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash"
LAYOUT_FILE="/tmp/voice_pick_terminator_layout.conf"
TERMINATOR_LOG="/tmp/voice_pick_terminator.log"

# 전부 bash -lc(로그인 셸)로 감싼다 - realsense 같은 ~/.bashrc alias까지 확실히 풀리게.
wrap() { echo "bash -lc 'echo \"$1\"; $2; exec bash'"; }

python3 "$WS_DIR/scripts/gen_terminator_layout.py" "$LAYOUT_FILE" \
  "0-RealSense"      "$(wrap 'RealSense 카메라 구동 중...' 'realsense')" \
  "1-Bringup"        "$(wrap '로봇+카메라 드라이버(가상모드) 기동 중...' "$ROS_ENV && ros2 launch m0609_rg2_bringup bringup_camera.launch.py")" \
  "2-MoveIt"         "$(wrap 'MoveIt 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 launch m0609_rg2_moveit moveit_camera.launch.py")" \
  "3-HandPublisher"  "$(wrap '손 위치 퍼블리셔 대기 중 (5초)...' "sleep 5; $ROS_ENV && cd $SCRIPTS_DIR && python3 hand_publisher.py")" \
  "4-HandAvoidance"  "$(wrap '손 회피 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run m0609_rg2_bringup hand_avoidance.py")" \
  "5-Backend"        "$(wrap 'HMI 백엔드(ROS2 브릿지 포함) 구동 중... (--reload 금지)' "export PYTHONUNBUFFERED=1; $ROS_ENV && cd $HOME/ws_cobot2_pjt/backend && python3 -m uvicorn app.main:app --port 8000")" \
  "6-Frontend"       "$(wrap 'HMI 프론트엔드 구동 중...' "cd $HOME/ws_cobot2_pjt/frontend && npm run dev")" \
  "7-get_keyword"    "$(wrap '음성 인식 노드 대기 중 (5초)...' "sleep 5; $ROS_ENV && ros2 run voice_processing get_keyword")" \
  "8-tool_pick_yolo" "$(wrap '도구 픽업 스크립트 대기 중 (8초)...' "sleep 8; $ROS_ENV && cd $SCRIPTS_DIR && python3 tool_pick_yolo_target.py")" \
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

echo "✅ 실행됐습니다! HMI: http://localhost:5173 (또는 5174) -> 로그인 -> 홈에서 음성 시작 버튼"
