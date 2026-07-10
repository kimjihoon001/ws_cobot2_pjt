"""ROS2 <-> HMI(FastAPI) 브릿지.

voice_processing의 get_keyword.py가 "이 도구들 맞아?"라고 confirm_tools 서비스로
물어보면, 그 질문을 웹소켓으로 화면에 띄우고 사람이 답할 때까지 기다렸다가
답을 서비스 응답으로 돌려준다.

A_2 프로젝트(~/ws_cobot_pjt_pj/A_2/backend/main.py)의 BridgeNode/_ros_spin/broadcast
패턴을 이 용도에 맞게 축소해서 재사용한다.
"""

import asyncio
import glob
import os
import json
import logging
import math
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dsr_msgs2.srv import GetRobotState, MoveStop, ServoOff, SetRobotControl
from od_msg.srv import ConfirmTools
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.srv import SetCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from .database import SessionLocal
from .models import VoiceConfirmRequest

log = logging.getLogger("ros_bridge")
ESTOP_STATES = {3, 5, 6, 10}  # SAFE_OFF, SAFE_STOP, EMERGENCY_STOP, SAFE_OFF2
ARDUINO_VID = 0x2341
CONVEYOR_DEFAULT_PORT = "/dev/ttyACM0"
HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
ROBOT_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

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
        self._jenga_inspection_running = False
        self._estop_active = False
        self._estop_releasing = False
        self._estop_message = ""
        self._state_poll_in_flight = False
        self._joint_positions: dict[str, float] = {}
        self._joint_units: dict[str, str] = {f"joint_{i}": "deg" for i in range(1, 7)}
        self._joint_units["gripper"] = "mm"
        self._last_joint_state_at = 0.0
        self._last_gripper_state_at = 0.0

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
        self._move_action_cancel_cli = self.create_client(
            CancelGoal, "/move_action/_action/cancel_goal", callback_group=cb_group
        )
        self._execute_trajectory_cancel_cli = self.create_client(
            CancelGoal, "/execute_trajectory/_action/cancel_goal", callback_group=cb_group
        )
        self._controller_trajectory_cancel_clients = []
        for service_name in (
            "/dsr01/dsr_moveit_controller/follow_joint_trajectory/_action/cancel_goal",
            "/dsr_moveit_controller/follow_joint_trajectory/_action/cancel_goal",
        ):
            self._controller_trajectory_cancel_clients.append(
                (
                    service_name,
                    self.create_client(CancelGoal, service_name, callback_group=cb_group),
                )
            )
        self._switch_controller_cli = self.create_client(
            SwitchController, "/dsr01/controller_manager/switch_controller", callback_group=cb_group
        )
        self._move_stop_cli = self.create_client(MoveStop, "/dsr01/motion/move_stop", callback_group=cb_group)
        self._servo_off_cli = self.create_client(ServoOff, "/dsr01/system/servo_off", callback_group=cb_group)
        self._set_control_cli = self.create_client(
            SetRobotControl, "/dsr01/system/set_robot_control", callback_group=cb_group
        )
        self._get_robot_state_cli = self.create_client(
            GetRobotState, "/dsr01/system/get_robot_state", callback_group=cb_group
        )
        self._gripper_command_cli = self.create_client(SetCommand, "/onrobot/sendCommand", callback_group=cb_group)
        self._joint_trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/dsr01/dsr_moveit_controller/follow_joint_trajectory",
            callback_group=cb_group,
        )
        self._pick_task_pub = self.create_publisher(String, "pick_task_tools", 10)
        self._hmi_stop_pub = self.create_publisher(String, "hmi/emergency_stop", 10)
        self._display_trajectory_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        for topic in ("/joint_states", "/dsr01/joint_states", "/dsr01/gz/joint_states", "/gripper_joint_states"):
            self.create_subscription(JointState, topic, self._handle_joint_state, 10, callback_group=cb_group)
        self.create_subscription(OnRobotRGInput, "/OnRobotRGInput", self._handle_gripper_status, 10, callback_group=cb_group)
        self.create_timer(0.5, self._poll_robot_state, callback_group=cb_group)
        self._hmi_get_keyword_in_flight = False
        self._last_pick_task_data = ""
        self._last_pick_task_at = 0.0
        self.get_logger().info("VoiceBridgeNode 준비 완료 (confirm_tools/confirm_release 서비스 대기 중)")

    def _handle_joint_state(self, msg: JointState):
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            if not math.isfinite(position):
                continue
            if name in {f"joint_{i}" for i in range(1, 7)}:
                self._joint_positions[name] = round(math.degrees(position), 2)
                self._joint_units[name] = "deg"
                self._last_joint_state_at = now
            elif name in {"finger_joint", "rg2_finger_joint", "gripper", "gripper_joint"}:
                self._joint_positions["gripper"] = round(self._rg2_joint_to_width_mm(position), 1)
                self._joint_units["gripper"] = "mm"
                self._last_gripper_state_at = now

    def _handle_gripper_status(self, msg: OnRobotRGInput):
        self._joint_positions["gripper"] = round(float(msg.gwdf) / 10.0, 1)
        self._joint_units["gripper"] = "mm"
        self._last_gripper_state_at = time.monotonic()

    def _poll_robot_state(self):
        if self._estop_releasing or self._state_poll_in_flight:
            return
        if not self._get_robot_state_cli.service_is_ready():
            return
        self._state_poll_in_flight = True
        future = self._get_robot_state_cli.call_async(GetRobotState.Request())
        future.add_done_callback(self._on_robot_state)

    def _on_robot_state(self, future):
        try:
            result = future.result()
            if result is None:
                return
            if result.robot_state in ESTOP_STATES and not self._estop_active:
                self._estop_active = True
                self._estop_message = f"외부 비상정지 감지 (robot_state={result.robot_state})"
                self._jenga_inspection_running = False
                self._hmi_get_keyword_in_flight = False
                self.get_logger().warning(self._estop_message)
        except Exception as e:
            self.get_logger().warning(f"로봇 상태 폴링 실패: {e}")
        finally:
            self._state_poll_in_flight = False

    @staticmethod
    def _rg2_joint_to_width_mm(joint_angle: float) -> float:
        l1 = 0.108505
        l3 = 0.055
        theta1 = 1.41371
        theta3 = 0.76794
        dy = -0.0144
        width_m = (math.cos(joint_angle + theta3) * l3 + dy + l1 * math.cos(theta1)) * 2
        return max(0.0, min(110.0, width_m * 1000.0))

    def start_listening(self) -> bool:
        """HMI '음성 시작' 버튼. get_keyword 서비스가 안 떠 있으면 False."""
        if self._estop_active:
            return False
        if not self._get_keyword_cli.service_is_ready():
            return False
        self._hmi_get_keyword_in_flight = True
        future = self._get_keyword_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_get_keyword_done)
        return True

    def start_jenga_inspection(self) -> bool:
        """HMI 직접 실행 버튼. 음성 명령 없이 젠가 품질검사를 시작한다."""
        if self._estop_active:
            return False
        if not self._jenga_inspection_cli.service_is_ready():
            return False
        self._jenga_inspection_running = True
        future = self._jenga_inspection_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_jenga_inspection_done)
        self.get_logger().info("HMI 직접 실행으로 젠가 품질 검사를 시작했습니다.")
        return True

    def deliver_hammer_screwdriver(self) -> bool:
        """HMI 직접 실행 버튼. 음성 명령 없이 hammer/screwdriver 전달 작업을 시작한다."""
        if self._estop_active:
            return False
        self._publish_pick_task_data("hammer:user screwdriver:user", "hmi_direct")
        return True

    def move_home(self) -> dict:
        """HMI 수동 제어: MoveIt 실행 컨트롤러로 기본 JReady 홈 자세에 보낸다."""
        if self._estop_active:
            return {"success": False, "message": "비상정지 상태에서는 홈 위치 이동을 실행할 수 없습니다."}
        positions = [math.radians(value) for value in HOME_JOINTS_DEG]
        return self._send_joint_trajectory(positions, duration_sec=5, label="홈 위치 이동")

    def open_gripper(self) -> dict:
        return self._send_gripper_command("o", "그리퍼 열기")

    def close_gripper(self) -> dict:
        return self._send_gripper_command("c", "그리퍼 닫기")

    def _send_gripper_command(self, command: str, label: str) -> dict:
        if self._estop_active:
            return {"success": False, "message": f"비상정지 상태에서는 {label}를 실행할 수 없습니다."}
        req = SetCommand.Request()
        req.command = command
        result = self._call_dsr_service(self._gripper_command_cli, req, label, timeout=2.0)
        if result["success"]:
            return {"success": True, "message": f"{label} 명령을 보냈습니다."}
        return self._public_service_result(result)

    def _send_joint_trajectory(self, positions: list[float], duration_sec: int, label: str) -> dict:
        if not self._joint_trajectory_action.wait_for_server(timeout_sec=1.0):
            return {"success": False, "message": "로봇 궤적 컨트롤러가 응답하지 않습니다."}

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ROBOT_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = RosDuration(sec=duration_sec)
        goal.trajectory.points = [point]

        done = threading.Event()
        result_box = {"accepted": False, "error_code": None, "error": None}
        send_future = self._joint_trajectory_action.send_goal_async(goal)

        def _goal_response(done_future):
            try:
                goal_handle = done_future.result()
                if not goal_handle.accepted:
                    done.set()
                    return
                result_box["accepted"] = True
                result_future = goal_handle.get_result_async()

                def _result(result_done_future):
                    try:
                        result_box["error_code"] = result_done_future.result().result.error_code
                    except Exception as exc:
                        result_box["error"] = exc
                    finally:
                        done.set()

                result_future.add_done_callback(_result)
            except Exception as exc:
                result_box["error"] = exc
                done.set()

        send_future.add_done_callback(_goal_response)
        if not done.wait(timeout=duration_sec + 3.0):
            return {"success": False, "message": f"{label} 응답 타임아웃"}
        if result_box["error"] is not None:
            return {"success": False, "message": f"{label} 실패: {result_box['error']}"}
        if not result_box["accepted"]:
            return {"success": False, "message": f"{label} 목표가 거부되었습니다."}
        if result_box["error_code"] != 0:
            return {"success": False, "message": f"{label} 실패(error_code={result_box['error_code']})"}
        return {"success": True, "message": f"{label} 완료"}

    def _call_dsr_service(self, client, request, label: str, timeout: float = 2.0) -> dict:
        if not client.wait_for_service(timeout_sec=timeout):
            message = f"{label} 서비스 응답 없음"
            self.get_logger().warning(message)
            return {"success": False, "message": message}

        done = threading.Event()
        result_box = {"result": None, "error": None}
        future = client.call_async(request)

        def _done_callback(done_future):
            try:
                result_box["result"] = done_future.result()
            except Exception as exc:
                result_box["error"] = exc
            finally:
                done.set()

        future.add_done_callback(_done_callback)
        if not done.wait(timeout=timeout):
            message = f"{label} 응답 타임아웃"
            self.get_logger().warning(message)
            return {"success": False, "message": message}
        if result_box["error"] is not None:
            message = f"{label} 호출 실패: {result_box['error']}"
            self.get_logger().error(message)
            return {"success": False, "message": message}

        result = result_box["result"]
        success = bool(getattr(result, "success", True))
        message = getattr(result, "message", label)
        return {"success": success, "message": str(message), "response": result}

    @staticmethod
    def _public_service_result(result: dict) -> dict:
        return {"success": result["success"], "message": result["message"]}

    def emergency_stop(self) -> dict:
        """HMI 긴급정지. MoveIt 목표 취소, 작업 노드 중단 신호, controller 정지를 요청한다."""
        self._jenga_inspection_running = False
        self._hmi_get_keyword_in_flight = False
        self._estop_active = True
        self._estop_message = "HMI 긴급정지 활성화"
        self.get_logger().warning("HMI 긴급정지 요청 수신")

        stop_msg = String()
        stop_msg.data = "stop"
        for _ in range(3):
            self._hmi_stop_pub.publish(stop_msg)

        action_cancel = self._cancel_motion_goals()
        self._clear_displayed_trajectory()
        controller_stop = self._switch_moveit_controller(active=False)

        return {
            "success": True,
            "estop": self._estop_active,
            "message": "HMI 긴급정지 활성화",
            "details": {
                "action_cancel": action_cancel,
                "controller_stop": controller_stop,
            },
        }

    def release_estop(self) -> dict:
        """HMI 긴급정지 해제. MoveIt controller를 다시 활성화하고 작업 노드 중단을 해제한다."""
        self._estop_releasing = True
        self._estop_message = "HMI 긴급정지 해제 중"
        self.get_logger().info("HMI 긴급정지 해제 요청 수신")
        try:
            action_cancel = self._cancel_motion_goals()
            self._clear_displayed_trajectory()
            controller_start = self._switch_moveit_controller(active=True)

            release_msg = String()
            release_msg.data = "release"
            for _ in range(3):
                self._hmi_stop_pub.publish(release_msg)

            self._estop_active = False
            self._estop_message = "HMI 긴급정지 해제 완료"

            return {
                "success": True,
                "estop": self._estop_active,
                "message": self._estop_message,
                "details": {
                    "action_cancel": action_cancel,
                    "controller_start": controller_start,
                },
            }
        finally:
            self._estop_releasing = False

    def _cancel_action_goals(self, client, label: str, timeout: float = 1.0) -> dict:
        req = CancelGoal.Request()
        if not client.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": f"{label} 서비스 없음"}
        result = self._call_dsr_service(client, req, label, timeout=timeout)
        response = result.get("response")
        return {
            "success": result["success"],
            "message": f"return_code={getattr(response, 'return_code', 'unknown')}",
        }

    def _cancel_motion_goals(self) -> dict:
        results = {
            "move_action": self._cancel_action_goals(self._move_action_cancel_cli, "move_action cancel"),
            "execute_trajectory": self._cancel_action_goals(
                self._execute_trajectory_cancel_cli,
                "execute_trajectory cancel",
            ),
            "controller_trajectory": self._cancel_controller_trajectory_goals(),
        }
        # CancelGoal is best-effort; repeat once so a goal accepted during the first cancel window is caught.
        time.sleep(0.05)
        results["execute_trajectory_retry"] = self._cancel_action_goals(
            self._execute_trajectory_cancel_cli,
            "execute_trajectory cancel retry",
            timeout=0.5,
        )
        return results

    def _cancel_controller_trajectory_goals(self) -> list[dict]:
        results = []
        for service_name, client in self._controller_trajectory_cancel_clients:
            results.append(self._cancel_action_goals(client, service_name, timeout=0.2))
        return results

    def _clear_displayed_trajectory(self):
        empty_display = DisplayTrajectory()
        for _ in range(3):
            self._display_trajectory_pub.publish(empty_display)

    def _switch_moveit_controller(self, active: bool, timeout: float = 0.5) -> dict:
        req = SwitchController.Request()
        controller_name = "dsr_moveit_controller"
        req.activate_controllers = [controller_name] if active else []
        req.deactivate_controllers = [] if active else [controller_name]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout.sec = 1
        if not self._switch_controller_cli.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": "controller_manager switch_controller 서비스 없음"}
        result = self._call_dsr_service(
            self._switch_controller_cli,
            req,
            "controller_start" if active else "controller_stop",
            timeout=1.0,
        )
        response = result.get("response")
        ok = bool(getattr(response, "ok", False))
        return {"success": ok, "message": "ok" if ok else result["message"]}

    def _wait_until_not_estop(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            req = GetRobotState.Request()
            result = self._call_dsr_service(self._get_robot_state_cli, req, "get_robot_state", timeout=1.0)
            response = result.get("response")
            if response is not None and getattr(response, "robot_state", None) not in ESTOP_STATES:
                return True
            if not result["success"]:
                time.sleep(0.2)
                continue
            time.sleep(0.2)
        return False

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
        self._jenga_inspection_running = False

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

    def _is_conveyor_available(self) -> bool:
        try:
            import serial.tools.list_ports

            for port in serial.tools.list_ports.comports():
                if port.vid == ARDUINO_VID:
                    return True
        except Exception:
            pass
        return os.path.exists(CONVEYOR_DEFAULT_PORT) or bool(glob.glob("/dev/ttyACM*") or glob.glob("/dev/ttyUSB*"))

    def get_robot_status(self) -> dict:
        service_names = set()
        action_names = set()
        topic_names = set()
        try:
            service_names = {name for name, _ in self.get_service_names_and_types()}
            action_names = {name for name, _ in get_action_names_and_types(self)}
            topic_names = {name for name, _ in self.get_topic_names_and_types()}
        except Exception as e:
            self.get_logger().warn(f"로봇 상태 조회 실패: {e}")

        moveit_available = (
            "/move_action" in action_names
            or "/move_action/_action/status" in topic_names
            or "/move_action/_action/send_goal" in service_names
        )
        checks = {
            "dsr": "/dsr01/system/get_robot_state" in service_names,
            "moveit": moveit_available,
            "conveyor": self._is_conveyor_available(),
            "jenga_inspector": "/run_jenga_inspection" in service_names,
            "tool_pick": self.count_subscribers("pick_task_tools") > 0,
            "voice": "get_keyword" in service_names or "/get_keyword" in service_names,
            "hand": "/get_hand_position" in service_names,
        }
        if time.monotonic() - self._last_joint_state_at < 5.0:
            checks["dsr"] = True
        connected = checks["dsr"] and checks["moveit"]

        current_task = "대기"
        task_key = "idle"
        if self._estop_releasing:
            current_task = "비상정지 해제 중"
            task_key = "estop_releasing"
        elif self._estop_active:
            current_task = "비상정지"
            task_key = "estop"
        elif self._jenga_inspection_running:
            current_task = "품질검사 진행 중"
            task_key = "qc_running"
        elif self._pending:
            current_task = "도구 확인 대기"
            task_key = "tool_confirm"
        elif self._pending_release:
            current_task = "배송 확인 대기"
            task_key = "release_confirm"
        elif self._hmi_get_keyword_in_flight:
            current_task = "음성 명령 대기"
            task_key = "voice_listening"
        elif time.monotonic() - self._last_pick_task_at < 20.0:
            current_task = "도구 전달 진행 중"
            task_key = "tool_delivery"

        return {
            "connected": connected,
            "mode": "real" if checks["dsr"] else "unknown",
            "controller": "MoveIt" if checks["moveit"] else "미연결",
            "current_task": current_task,
            "task_key": task_key,
            "checks": checks,
            "estop": self._estop_active,
            "estop_message": self._estop_message,
            "joints": {name: self._joint_positions.get(name) for name in [*(f"joint_{i}" for i in range(1, 7)), "gripper"]},
            "joint_units": dict(self._joint_units),
            "last_pick_task": self._last_pick_task_data,
            "ros_bridge": True,
        }


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
