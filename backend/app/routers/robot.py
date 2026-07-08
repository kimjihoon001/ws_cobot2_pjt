from fastapi import APIRouter

from .. import ros_bridge

router = APIRouter(prefix="/api/robot", tags=["robot"])


@router.post("/start_listen")
def start_listen():
    """HMI '음성 시작' 버튼 -> get_keyword를 호출해 웨이크워드 대기를 켠다.
    확정되면 자동으로 tool_pick_yolo_target.py가 이어서 픽업을 진행한다."""
    listening = ros_bridge._bridge.start_listening() if ros_bridge._bridge else False
    return {"listening": listening}
