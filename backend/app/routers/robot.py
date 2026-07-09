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
                "jenga_inspector": False,
                "tool_pick": False,
                "voice": False,
                "hand": False,
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
    return {"started": started}


@router.post("/deliver_hammer_screwdriver")
def deliver_hammer_screwdriver():
    """HMI 직접 실행 버튼 -> 음성 명령 없이 hammer/screwdriver 전달을 시작한다."""
    started = ros_bridge._bridge.deliver_hammer_screwdriver() if ros_bridge._bridge else False
    return {"started": started}
