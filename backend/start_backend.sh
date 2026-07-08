#!/bin/bash
# HMI 백엔드 실행 스크립트.
#
# 주의: 이 백엔드는 반드시 이 스크립트로, 일반 bash 백그라운드 프로세스로 실행해야 합니다.
# (Claude Code의 preview_start 도구로 띄우면 ROS2 서비스 요청의 string[] 필드가
#  빈 배열로 오는 문제가 실제로 확인되어서, 지금은 bash로 직접 띄우는 방식만 사용합니다.)
#
# ~/cobot_ws에 있는 예전 od_msg(ConfirmTools 없음)가 환경에 섞여 들어오는 것도
# 별개로 정리해둔다 (원인은 아니었지만 계속 두면 헷갈리므로).
set -e

export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=67
source /opt/ros/humble/setup.bash
source /home/rokey/Downloads/ws_cobot2_pjt-voice_process/install/setup.bash

strip_cobot_ws() {
    echo "$1" | tr ':' '\n' | grep -v '/home/rokey/cobot_ws' | paste -sd ':' -
}

export AMENT_PREFIX_PATH="$(strip_cobot_ws "$AMENT_PREFIX_PATH")"
export LD_LIBRARY_PATH="$(strip_cobot_ws "$LD_LIBRARY_PATH")"
export PYTHONPATH="$(strip_cobot_ws "$PYTHONPATH")"
export ROS_PACKAGE_PATH="$(strip_cobot_ws "$ROS_PACKAGE_PATH")"
export CMAKE_PREFIX_PATH="$(strip_cobot_ws "$CMAKE_PREFIX_PATH")"
export COLCON_PREFIX_PATH="$(strip_cobot_ws "$COLCON_PREFIX_PATH")"

cd /home/rokey/ws_cobot2_pjt-web-hmi/backend
exec python3 -m uvicorn app.main:app --port 8000
