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
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger
from od_msg.srv import ConfirmTools

from .database import SessionLocal
from .models import VoiceConfirmRequest

log = logging.getLogger("ros_bridge")

_clients: set = set()                     # 지금 연결된 웹소켓들
_loop: asyncio.AbstractEventLoop = None    # ROS2 스레드에서 웹소켓으로 보낼 때 필요한 asyncio 루프
_bridge = None                             # VoiceBridgeNode 인스턴스


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class VoiceBridgeNode(Node):
    """get_keyword.py(confirm_tools)와 tool_pick_yolo_target.py(confirm_release)가
    물어보면(서비스 서버), DB에 pending row를 남기고 화면에 띄운 뒤 답 올 때까지 기다린다.
    HMI는 DB를 폴링해서 대기 중인 요청을 읽고, REST로 답을 제출한다 (WS는 "새 요청 생김"
    즉시 알림 용도로만 병행 — 정답 소스는 DB)."""

    def __init__(self):
        super().__init__("voice_confirm_bridge")
        self._pending = None          # confirm_tools 대기 슬롯 (한 번에 하나만 가능)
        self._pending_release = None  # confirm_release 대기 슬롯 (한 번에 하나만 가능)

        # confirm_tools/confirm_release 콜백은 각각 최대 60초 threading.Event().wait()로
        # 블로킹된다. 실행기가 SingleThreadedExecutor면 그동안 다른 콜백(다른 쪽 확인 요청,
        # get_keyword 응답 콜백 등)이 전혀 처리 못 되고 그 60초를 그대로 기다려야 하는
        # head-of-line blocking이 생김 - MultiThreadedExecutor + ReentrantCallbackGroup으로
        # 콜백들이 서로를 막지 않고 동시에 처리되게 한다.
        cb_group = ReentrantCallbackGroup()
        self.create_service(ConfirmTools, "confirm_tools", self._handle_confirm, callback_group=cb_group)
        self.create_service(ConfirmTools, "confirm_release", self._handle_confirm_release, callback_group=cb_group)

        # HMI '음성 시작' 버튼 -> 여기가 /get_keyword를 호출(클라이언트)해서 웨이크워드 대기를
        # 켠다. 응답이 오면 확정된 도구 목록을 /pick_task_tools 토픽으로 발행해서
        # tool_pick_yolo_target.py가 집어가게 한다 (get_keyword를 부르는 주체는 여기 하나뿐).
        self._get_keyword_cli = self.create_client(Trigger, "get_keyword", callback_group=cb_group)
        self._jenga_inspection_cli = self.create_client(Trigger, "/run_jenga_inspection", callback_group=cb_group)
        self._pick_task_pub = self.create_publisher(String, "pick_task_tools", 10)
        self._hmi_alert_sub = self.create_subscription(String, "hmi_alert", self._hmi_alert_cb, 10, callback_group=cb_group)
        self._hmi_get_keyword_in_flight = False
        self._last_pick_task_data = ""
        self._last_pick_task_at = 0.0
        self.get_logger().info("VoiceBridgeNode 준비 완료 (confirm_tools/confirm_release 서비스 대기 중)")

    def start_listening(self) -> bool:
        """HMI '음성 시작' 버튼. get_keyword 서비스가 안 떠 있으면 False."""
        if not self._get_keyword_cli.service_is_ready():
            return False
        self._hmi_get_keyword_in_flight = True
        future = self._get_keyword_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_get_keyword_done)
        return True

    def start_jenga_inspection(self) -> bool:
        """HMI 직접 실행 버튼. 음성 명령 없이 젠가 품질검사를 시작한다."""
        if not self._jenga_inspection_cli.service_is_ready():
            return False
        future = self._jenga_inspection_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_jenga_inspection_done)
        self.get_logger().info("HMI 직접 실행으로 젠가 품질 검사를 시작했습니다.")
        return True

    def deliver_hammer_screwdriver(self) -> bool:
        """HMI 직접 실행 버튼. 음성 명령 없이 hammer/screwdriver 전달 작업을 시작한다."""
        self._publish_pick_task_data("hammer:user screwdriver:user", "hmi_direct")
        return True

    def _publish_pick_task(self, tools, targets, source: str):
        data = " ".join(f"{tool}:{target}" for tool, target in zip(tools, targets))
        self._publish_pick_task_data(data, source)

    def _publish_pick_task_data(self, data: str, source: str):
        now = time.monotonic()
        if data == self._last_pick_task_data and now - self._last_pick_task_at < 5.0:
            self.get_logger().info(f"/pick_task_tools 중복 발행 생략({source}): {data!r}")
            return
        msg = String()
        msg.data = data
        self._pick_task_pub.publish(msg)
        self._last_pick_task_data = data
        self._last_pick_task_at = now
        self.get_logger().info(f"/pick_task_tools 발행({source}): {data!r}")

    def _on_get_keyword_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"get_keyword 호출 실패: {e}")
            self._hmi_get_keyword_in_flight = False
            return
        if result is None or not result.success:
            self.get_logger().info("get_keyword 응답 없음/실패 - 픽업 트리거 안 함")
            self._hmi_get_keyword_in_flight = False
            return
        self.get_logger().info(f"get_keyword 응답 수신: {result.message!r}")
        self._publish_pick_task_data(result.message, "get_keyword_done")
        self._hmi_get_keyword_in_flight = False

    def _on_jenga_inspection_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"run_jenga_inspection 호출 실패: {e}")
            return
        if result is None:
            self.get_logger().error("run_jenga_inspection 응답 없음")
            return
        if result.success:
            self.get_logger().info(f"run_jenga_inspection 완료: {result.message}")
        else:
            self.get_logger().error(f"run_jenga_inspection 실패: {result.message}")

    def _start_pending(self, kind: str, tools, targets) -> dict:
        """DB에 pending row 생성 + 화면에 띄울 payload 구성."""
        db = SessionLocal()
        try:
            row = VoiceConfirmRequest(
                kind=kind,
                tools=json.dumps(list(tools), ensure_ascii=False),
                targets=json.dumps(list(targets), ensure_ascii=False),
                status="pending",
            )
            db.add(row)
            db.commit()
            db.refresh(row)
            db_id = row.id
        finally:
            db.close()

        payload = {
            "type": "confirm_request" if kind == "tool_confirm" else "release_request",
            "id": db_id,
            "tools": list(tools),
            "targets": list(targets),
        }
        return {
            "event": threading.Event(),
            "result": {"confirmed": False},
            "payload": payload,
            "db_id": db_id,
        }

    def _resolve(self, pending: dict | None, confirmed: bool):
        if not pending:
            return
        db = SessionLocal()
        try:
            row = db.get(VoiceConfirmRequest, pending["db_id"])
            if row is not None:
                row.status = "confirmed" if confirmed else "rejected"
                row.resolved_at = _utcnow()
                db.commit()
        finally:
            db.close()
        pending["result"]["confirmed"] = confirmed
        pending["event"].set()

    def _handle_confirm(self, request, response):
        pending = self._start_pending("tool_confirm", request.tools, request.targets)
        self._pending = pending
        self.get_logger().info(f"확인 요청 수신: {pending['payload']}")

        # 화면(웹소켓)에 "확인해달라"고 알림 (ROS2 스레드 -> asyncio 쪽으로 안전하게 넘김)
        if _loop:
            asyncio.run_coroutine_threadsafe(broadcast(pending["payload"]), _loop)
        else:
            self.get_logger().error("asyncio 루프가 없어서 화면에 알릴 수 없습니다.")

        got_answer = pending["event"].wait(timeout=60.0)  # 최대 60초 대기
        response.confirmed = pending["result"]["confirmed"] if got_answer else False
        if response.confirmed:
            self._publish_pick_task(request.tools, request.targets, "confirm_tools")
        self._pending = None
        return response

    def _handle_confirm_release(self, request, response):
        pending = self._start_pending("release_confirm", request.tools, request.targets)
        self._pending_release = pending
        self.get_logger().info(f"배송 확인 요청 수신: {pending['payload']}")

        if _loop:
            asyncio.run_coroutine_threadsafe(broadcast(pending["payload"]), _loop)
        else:
            self.get_logger().error("asyncio 루프가 없어서 화면에 알릴 수 없습니다.")

        got_answer = pending["event"].wait(timeout=60.0)
        response.confirmed = pending["result"]["confirmed"] if got_answer else False
        self._pending_release = None
        return response

    def resolve_pending(self, confirmed: bool):
        """웹소켓으로 사람 답이 도착했을 때 호출됨 (main.py의 /ws 엔드포인트에서 호출)."""
        self._resolve(self._pending, confirmed)

    def resolve_pending_release(self, confirmed: bool):
        self._resolve(self._pending_release, confirmed)

    def resolve_by_id(self, db_id: int, confirmed: bool) -> bool:
        """HMI가 REST로 답을 제출할 때 사용 (row id 기준으로 어느 슬롯이든 찾아서 해결)."""
        if self._pending and self._pending["db_id"] == db_id:
            self._resolve(self._pending, confirmed)
            return True
        if self._pending_release and self._pending_release["db_id"] == db_id:
            self._resolve(self._pending_release, confirmed)
            return True

        db = SessionLocal()
        try:
            row = db.get(VoiceConfirmRequest, db_id)
            if row is None or row.status != "pending":
                return False

            row.status = "confirmed" if confirmed else "rejected"
            row.resolved_at = _utcnow()
            tools = json.loads(row.tools)
            targets = json.loads(row.targets)
            kind = row.kind
            db.commit()
        finally:
            db.close()

        self.get_logger().warning(
            f"라이브 pending 슬롯 없이 DB 요청만 처리됨: id={db_id}, confirmed={confirmed}"
        )
        if confirmed and kind == "tool_confirm":
            self._publish_pick_task(tools, targets, "stale_tool_confirm")
        return True

    def get_pending_payload(self):
        """작업 화면이 늦게 열렸을 때, 아직 대기 중인 확인 요청을 다시 보낸다."""
        return self._pending["payload"] if self._pending else None

    def get_pending_release_payload(self):
        return self._pending_release["payload"] if self._pending_release else None

    def _hmi_alert_cb(self, msg: String):
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                broadcast({"type": "alert", "message": msg.data}),
                self._loop
            )


def _ros_spin():
    while True:
        try:
            executor = MultiThreadedExecutor(num_threads=4)
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
