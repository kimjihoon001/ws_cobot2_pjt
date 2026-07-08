#!/bin/bash

# ==============================================================================
# 젠가 불량 검사, 음성 명령 및 HMI 통합 시스템 자동 실행 스크립트
# ==============================================================================

echo "🚀 젠가 불량 검사 시스템을 시작합니다... Terminator 분할 화면이 열립니다."

# 워크스페이스 경로 자동 감지
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

TERMINATOR_LOG="/tmp/jenga_terminator.log"
terminator --no-dbus --maximise --config "$WS_DIR/terminator_jenga.config" --layout jenga \
    >"$TERMINATOR_LOG" 2>&1 &
TERMINATOR_PID=$!

sleep 1
if ! kill -0 "$TERMINATOR_PID" 2>/dev/null; then
    echo "❌ Terminator 실행에 실패했습니다."
    cat "$TERMINATOR_LOG"
    exit 1
fi

disown "$TERMINATOR_PID" 2>/dev/null || true

echo "✅ 모든 프로세스가 Terminator 분할 화면에서 실행되었습니다."
echo "Terminator 로그: $TERMINATOR_LOG"
