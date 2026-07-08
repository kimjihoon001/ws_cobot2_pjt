#!/bin/bash
# get_keyword 노드 실행 스크립트.
#
# 주의: ~/.bashrc가 새 터미널마다 ~/cobot_ws/install/setup.bash를 자동으로 source해서,
# 그 안의 예전 od_msg(ConfirmTools 없는 버전)가 환경에 섞여 들어옵니다.
# 그 상태로 voice_process 워크스페이스를 source해도 안 씻겨서, confirm_tools 서비스
# 호출 시 "failed to create array for field 'tools'" 에러가 납니다.
# (HMI 백엔드 쪽 start_backend.sh에서 이미 겪고 고쳤던 것과 같은 문제라, 여기도 같은
#  방식으로 ~/cobot_ws 경로를 걸러낸다.)
set -e

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

exec ros2 run voice_processing get_keyword
