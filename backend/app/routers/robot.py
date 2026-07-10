from fastapi import APIRouter

from .. import ros_bridge

router = APIRouter(prefix="/api/robot", tags=["robot"])


@router.get("/status")
def robot_status():
    if not ros_bridge._bridge:
        return {
            "connected": False,
            "mode": "unknown",
            "controller": "미연결",
            "current_task": "대기",
            "task_key": "idle",
            "checks": {
                "dsr": False,
                "moveit": False,
                "conveyor": False,
                "jenga_inspector": False,
                "tool_pick": False,
                "voice": False,
                "hand": False,
            },
            "estop": False,
            "estop_message": "",
            "joints": {
                "joint_1": None,
                "joint_2": None,
                "joint_3": None,
                "joint_4": None,
                "joint_5": None,
                "joint_6": None,
                "gripper": None,
            },
            "joint_units": {
                "joint_1": "deg",
                "joint_2": "deg",
                "joint_3": "deg",
                "joint_4": "deg",
                "joint_5": "deg",
                "joint_6": "deg",
                "gripper": "mm",
            },
            "last_pick_task": "",
            "ros_bridge": False,
        }
    return ros_bridge._bridge.get_robot_status()


@router.post("/start_listen")
def start_listen():
    """HMI '음성 시작' 버튼 -> get_keyword를 호출해 웨이크워드 대기를 켠다.
    확정되면 자동으로 tool_pick_yolo_target.py가 이어서 픽업을 진행한다."""
    listening = ros_bridge._bridge.start_listening() if ros_bridge._bridge else False
    return {"listening": listening}


@router.post("/run_inspection")
def run_inspection():
    """HMI 직접 실행 버튼 -> 음성 명령 없이 젠가 품질검사를 시작한다."""
    started = ros_bridge._bridge.start_jenga_inspection() if ros_bridge._bridge else False
    return {"started": started, "message": "젠가 재검사를 시작했습니다." if started else "검사 노드 없음"}


@router.post("/deliver_hammer_screwdriver")
def deliver_hammer_screwdriver():
    """HMI 직접 실행 버튼 -> 음성 명령 없이 hammer/screwdriver 전달을 시작한다."""
    started = ros_bridge._bridge.deliver_hammer_screwdriver() if ros_bridge._bridge else False
    return {"started": started}


@router.post("/retry_pick_task")
def retry_pick_task():
    """HMI 예외 팝업 -> 마지막 도구 작업을 다시 스캔/배송한다."""
    if not ros_bridge._bridge:
        return {"success": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.retry_last_pick_task()


@router.post("/cancel_task")
def cancel_task():
    """HMI 예외 팝업 -> 현재 작업 취소 및 홈 복귀 요청."""
    if not ros_bridge._bridge:
        return {"success": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.cancel_task()


@router.post("/move_home")
def move_home():
    """HMI 수동 제어 -> 기본 홈 위치(JReady) 이동."""
    if not ros_bridge._bridge:
        return {"success": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.move_home()


@router.post("/open_gripper")
def open_gripper():
    """HMI 수동 제어 -> OnRobot RG2 열기."""
    if not ros_bridge._bridge:
        return {"success": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.open_gripper()


@router.post("/close_gripper")
def close_gripper():
    """HMI 수동 제어 -> OnRobot RG2 닫기."""
    if not ros_bridge._bridge:
        return {"success": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.close_gripper()


@router.post("/emergency_stop")
def emergency_stop():
    """HMI 비상정지 -> DSR move_stop + servo_off 요청."""
    if not ros_bridge._bridge:
        return {"success": False, "estop": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.emergency_stop()


@router.post("/release_estop")
def release_estop():
    """HMI 비상정지 해제 -> safe stop reset + servo on 요청."""
    if not ros_bridge._bridge:
        return {"success": False, "estop": False, "message": "ROS 브릿지 미연결"}
    return ros_bridge._bridge.release_estop()
