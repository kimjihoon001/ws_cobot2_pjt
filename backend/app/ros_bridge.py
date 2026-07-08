"""ROS2 <-> HMI(FastAPI) 브릿지.

voice_processing의 get_keyword.py가 "이 도구들 맞아?"라고 confirm_tools 서비스로
물어보면, 그 질문을 웹소켓으로 화면에 띄우고 사람이 답할 때까지 기다렸다가
답을 서비스 응답으로 돌려준다.

A_2 프로젝트(~/ws_cobot_pjt_pj/A_2/backend/main.py)의 BridgeNode/_ros_spin/broadcast
패턴을 이 용도에 맞게 축소해서 재사용한다.
"""

import asyncio
import json
import logging
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from od_msg.srv import ConfirmTools

log = logging.getLogger("ros_bridge")

_clients: set = set()                     # 지금 연결된 웹소켓들
_loop: asyncio.AbstractEventLoop = None    # ROS2 스레드에서 웹소켓으로 보낼 때 필요한 asyncio 루프
_bridge = None                             # VoiceBridgeNode 인스턴스


class VoiceBridgeNode(Node):
    """get_keyword.py가 물어보면(서비스 서버), 화면에 띄우고 답 올 때까지 기다린다."""

    def __init__(self):
        super().__init__("voice_confirm_bridge")
        self._pending = None  # 지금 화면에서 답 기다리는 중인 요청 (한 번에 하나만 가능)
        self.create_service(ConfirmTools, "confirm_tools", self._handle_confirm)
        self.get_logger().info("VoiceBridgeNode 준비 완료 (confirm_tools 서비스 대기 중)")

    def _handle_confirm(self, request, response):
        event = threading.Event()
        result_box = {"confirmed": False}

        payload = {
            "type": "confirm_request",
            "tools": list(request.tools),
            "targets": list(request.targets),
        }
        self._pending = {"event": event, "result": result_box, "payload": payload}
        self.get_logger().info(f"확인 요청 수신: {payload}")

        # 화면(웹소켓)에 "확인해달라"고 알림 (ROS2 스레드 -> asyncio 쪽으로 안전하게 넘김)
        if _loop:
            asyncio.run_coroutine_threadsafe(broadcast(payload), _loop)
        else:
            self.get_logger().error("asyncio 루프가 없어서 화면에 알릴 수 없습니다.")

        got_answer = event.wait(timeout=60.0)  # 최대 60초 대기
        response.confirmed = result_box["confirmed"] if got_answer else False
        self._pending = None
        return response

    def resolve_pending(self, confirmed: bool):
        """웹소켓으로 사람 답이 도착했을 때 호출됨 (main.py의 /ws 엔드포인트에서 호출)."""
        if self._pending:
            self._pending["result"]["confirmed"] = confirmed
            self._pending["event"].set()

    def get_pending_payload(self):
        """작업 화면이 늦게 열렸을 때, 아직 대기 중인 확인 요청을 다시 보낸다."""
        if self._pending:
            return self._pending.get("payload")
        return None


def _ros_spin():
    while True:
        try:
            executor = SingleThreadedExecutor()
            executor.add_node(_bridge)
            executor.spin()
        except BaseException as e:
            log.error(f"ROS2 spin 예외 (1초 후 재시작): {e}")
            time.sleep(1)


async def broadcast(data: dict):
    if not _clients:
        log.warning(f"연결된 HMI WebSocket이 없어 확인 요청을 보류함: {data}")
        return
    msg = json.dumps(data, ensure_ascii=False)
    dead = set()
    for ws in _clients:
        try:
            await ws.send_text(msg)
        except Exception:
            dead.add(ws)
    _clients.difference_update(dead)


def start_bridge(loop):
    """FastAPI 시작할 때 호출: ROS2 초기화 + 노드 생성 + 백그라운드 스레드로 spin 시작."""
    global _bridge, _loop
    _loop = loop
    rclpy.init()
    _bridge = VoiceBridgeNode()
    threading.Thread(target=_ros_spin, daemon=True, name="VoiceRosBridge").start()
    log.info("ROS2 브릿지 노드 시작")
    return _bridge
