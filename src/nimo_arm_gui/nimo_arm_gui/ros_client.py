"""Thin rclpy client for nimo_arm_ros services/topics/actions."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from nimo_arm_msgs.action import MovJ, MovL
from nimo_arm_msgs.msg import DualArmStatus, JogCart, JogJoint, ServoPByPico
from nimo_arm_msgs.srv import (
    RecordStart,
    RecordStop,
    SetBrake,
    SetCartImpKd,
    SetControlMode,
    SetJointImpKd,
    SetTeleopMotion,
    TrajList,
    TrajPostprocess,
    TrajRecordStart,
    TrajRecordStop,
    TrajReplayStart,
    TrajReplayStop,
    WaypointDelete,
    WaypointGoto,
    WaypointList,
    WaypointSave,
)

OP_NAMES = [
    "Unconfigured",
    "Standby",
    "Enabling",
    "Ready",
    "Moving",
    "Guiding",
    "Disabling",
    "Fault",
    "EStop",
    "Finalized",
]

MODE_NAMES = ["Position", "JointImp", "CartImp", "Drag"]


class ArmRosClient(Node):
    def __init__(self, ns: str = "nimo_arm") -> None:
        super().__init__("nimo_arm_hmi")
        self.ns = "/" + ns.strip("/")
        self.status: Optional[DualArmStatus] = None
        self.joint_state: Optional[JointState] = None
        self._status_cb: Optional[Callable[[DualArmStatus], None]] = None
        # rokea_drag_bridge heartbeat (std_msgs/Bool: true = connected+drag_on)
        self.drag_bridge_ok: bool = False
        self._drag_bridge_stamp: float = 0.0

        self.create_subscription(DualArmStatus, f"{self.ns}/status", self._on_status, 10)
        self.create_subscription(JointState, f"{self.ns}/joint_states", self._on_js, 10)
        self.create_subscription(
            Bool, "rokea/drag_bridge/status", self._on_drag_bridge, 10
        )

        self._cli_cache: dict[str, object] = {}
        self._mov_j = ActionClient(self, MovJ, f"{self.ns}/mov_j")
        self._mov_l = ActionClient(self, MovL, f"{self.ns}/mov_l")

        self._pub_servo_j = {
            "left": self.create_publisher(JointState, f"{self.ns}/left/servo_j", 10),
            "right": self.create_publisher(JointState, f"{self.ns}/right/servo_j", 10),
        }
        self._pub_servo_p = {
            "left": self.create_publisher(PoseStamped, f"{self.ns}/left/servo_p", 10),
            "right": self.create_publisher(PoseStamped, f"{self.ns}/right/servo_p", 10),
        }
        self._pub_pico = {
            "left": self.create_publisher(ServoPByPico, f"{self.ns}/left/servo_p_by_pico", 10),
            "right": self.create_publisher(ServoPByPico, f"{self.ns}/right/servo_p_by_pico", 10),
        }
        self._pub_jog_j = {
            "left": self.create_publisher(JogJoint, f"{self.ns}/left/jog_j", 10),
            "right": self.create_publisher(JogJoint, f"{self.ns}/right/jog_j", 10),
        }
        self._pub_jog_c = {
            "left": self.create_publisher(JogCart, f"{self.ns}/left/jog_cart", 10),
            "right": self.create_publisher(JogCart, f"{self.ns}/right/jog_cart", 10),
        }

    def set_status_callback(self, cb: Callable[[DualArmStatus], None]) -> None:
        self._status_cb = cb

    def _on_status(self, msg: DualArmStatus) -> None:
        self.status = msg
        if self._status_cb:
            self._status_cb(msg)

    def _on_js(self, msg: JointState) -> None:
        self.joint_state = msg

    def _on_drag_bridge(self, msg: Bool) -> None:
        self.drag_bridge_ok = bool(msg.data)
        self._drag_bridge_stamp = time.monotonic()

    def drag_bridge_alive(self, timeout_s: float = 0.5) -> bool:
        """True if status received recently and data==true (drag on)."""
        if self._drag_bridge_stamp <= 0.0:
            return False
        if (time.monotonic() - self._drag_bridge_stamp) > timeout_s:
            return False
        return self.drag_bridge_ok

    def _trigger(self, name: str, timeout: float = 5.0) -> tuple[bool, str]:
        cli = self.create_client(Trigger, f"{self.ns}/{name}")
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, f"service {name} unavailable"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def call_init(self) -> tuple[bool, str]:
        return self._trigger("init")

    def call_shutdown(self) -> tuple[bool, str]:
        return self._trigger("shutdown")

    def call_go_home(self) -> tuple[bool, str]:
        return self._trigger("go_home")

    def call_go_work(self) -> tuple[bool, str]:
        return self._trigger("go_work")

    def call_arm_trigger(self, side: str, name: str) -> tuple[bool, str]:
        return self._trigger(f"{side}/{name}")

    def set_control_mode(self, side: str, mode: int) -> tuple[bool, str]:
        cli = self.create_client(SetControlMode, f"{self.ns}/{side}/set_control_mode")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = SetControlMode.Request()
        req.mode = mode
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def set_brake(self, side: str, command: int) -> tuple[bool, str]:
        cli = self.create_client(SetBrake, f"{self.ns}/{side}/set_brake")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = SetBrake.Request()
        req.command = command
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def set_joint_imp_kd(self, side: str, k: list[float], d: list[float]) -> tuple[bool, str]:
        cli = self.create_client(SetJointImpKd, f"{self.ns}/{side}/set_joint_imp_kd")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = SetJointImpKd.Request()
        req.k = list(k)
        req.d = list(d)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def set_cart_imp_kd(self, side: str, k: list[float], d: list[float]) -> tuple[bool, str]:
        cli = self.create_client(SetCartImpKd, f"{self.ns}/{side}/set_cart_imp_kd")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = SetCartImpKd.Request()
        req.k = list(k)
        req.d = list(d)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def publish_jog_j(self, side: str, joint: int, dq: float) -> None:
        msg = JogJoint()
        msg.joint = int(joint)
        msg.dq = float(dq)
        self._pub_jog_j[side].publish(msg)

    def publish_jog_cart(self, side: str, axis: int, rate: float) -> None:
        msg = JogCart()
        msg.axis = int(axis)
        msg.rate = float(rate)
        self._pub_jog_c[side].publish(msg)

    def publish_servo_j(self, side: str, q: list[float]) -> None:
        msg = JointState()
        msg.position = list(q)
        self._pub_servo_j[side].publish(msg)

    def publish_servo_p(self, side: str, pose: Pose) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "Link_Base"
        msg.pose = pose
        self._pub_servo_p[side].publish(msg)

    def publish_servo_p_by_pico(self, side: str, pose: Pose, is_run: bool) -> None:
        msg = ServoPByPico()
        msg.pose = pose
        msg.is_run = bool(is_run)
        self._pub_pico[side].publish(msg)

    def send_mov_j(
        self,
        move_left: bool,
        move_right: bool,
        left_q: list[float],
        right_q: list[float],
    ) -> tuple[bool, str]:
        if not self._mov_j.wait_for_server(timeout_sec=3.0):
            return False, "mov_j action unavailable"
        goal = MovJ.Goal()
        goal.move_left = move_left
        goal.move_right = move_right
        goal.left_q = list(left_q)
        goal.right_q = list(right_q)
        fut = self._mov_j.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, "goal rejected"
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
        if not res_fut.done() or res_fut.result() is None:
            return False, "timeout"
        wrapped = res_fut.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, getattr(wrapped.result, "message", "failed")
        return bool(wrapped.result.success), str(wrapped.result.message)

    def send_mov_l(
        self,
        move_left: bool,
        move_right: bool,
        left_pose: Pose,
        right_pose: Pose,
    ) -> tuple[bool, str]:
        if not self._mov_l.wait_for_server(timeout_sec=3.0):
            return False, "mov_l action unavailable"
        goal = MovL.Goal()
        goal.move_left = move_left
        goal.move_right = move_right
        goal.left_pose = left_pose
        goal.right_pose = right_pose
        fut = self._mov_l.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, "goal rejected"
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
        if not res_fut.done() or res_fut.result() is None:
            return False, "timeout"
        wrapped = res_fut.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, getattr(wrapped.result, "message", "failed")
        return bool(wrapped.result.success), str(wrapped.result.message)

    def waypoint_save(
        self, name: str, left: bool, right: bool, note: str = ""
    ) -> tuple[bool, str]:
        cli = self.create_client(WaypointSave, f"{self.ns}/waypoint/save")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = WaypointSave.Request()
        req.name = name
        req.capture_left = left
        req.capture_right = right
        req.note = note
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def waypoint_delete(self, name: str) -> tuple[bool, str]:
        cli = self.create_client(WaypointDelete, f"{self.ns}/waypoint/delete")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = WaypointDelete.Request()
        req.name = name
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def waypoint_list(self):
        cli = self.create_client(WaypointList, f"{self.ns}/waypoint/list")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", []
        fut = cli.call_async(WaypointList.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", []
        r = fut.result()
        return bool(r.success), str(r.message), list(r.waypoints)

    def waypoint_goto(self, name: str, left: bool, right: bool) -> tuple[bool, str]:
        cli = self.create_client(WaypointGoto, f"{self.ns}/waypoint/goto")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = WaypointGoto.Request()
        req.name = name
        req.move_left = left
        req.move_right = right
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def record_start(self, path: str = "", sample_period: float = 0.02) -> tuple[bool, str, str]:
        cli = self.create_client(RecordStart, f"{self.ns}/record/start")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", ""
        req = RecordStart.Request()
        req.path = path
        req.sample_period = sample_period
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", ""
        r = fut.result()
        return bool(r.success), str(r.message), str(r.path)

    def record_stop(self) -> tuple[bool, str, str, int]:
        cli = self.create_client(RecordStop, f"{self.ns}/record/stop")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", "", 0
        fut = cli.call_async(RecordStop.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", "", 0
        r = fut.result()
        return bool(r.success), str(r.message), str(r.path), int(r.rows)

    def set_teleop_motion(
        self, enable: bool, mode: str = "pico"
    ) -> tuple[bool, str]:
        cli = self.create_client(SetTeleopMotion, f"{self.ns}/set_teleop_motion")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        req = SetTeleopMotion.Request()
        req.enable = bool(enable)
        if mode == "servoj":
            req.mode = SetTeleopMotion.Request.MODE_SERVOJ
        else:
            req.mode = SetTeleopMotion.Request.MODE_PICO
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)

    def traj_record_start(
        self, path: str = "", sample_period: float = 0.02
    ) -> tuple[bool, str, str]:
        cli = self.create_client(TrajRecordStart, f"{self.ns}/traj/record/start")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", ""
        req = TrajRecordStart.Request()
        req.path = path
        req.sample_period = float(sample_period)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", ""
        r = fut.result()
        return bool(r.success), str(r.message), str(r.path)

    def traj_record_stop(self) -> tuple[bool, str, str, int]:
        cli = self.create_client(TrajRecordStop, f"{self.ns}/traj/record/stop")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", "", 0
        fut = cli.call_async(TrajRecordStop.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", "", 0
        r = fut.result()
        return bool(r.success), str(r.message), str(r.path), int(r.rows)

    def traj_postprocess(
        self,
        input_path: str,
        output_path: str = "",
        sg_window: int = 0,
        resample_dt: float = 0.0,
    ) -> tuple[bool, str, str, int]:
        cli = self.create_client(TrajPostprocess, f"{self.ns}/traj/postprocess")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", "", 0
        req = TrajPostprocess.Request()
        req.input_path = input_path
        req.output_path = output_path
        req.sg_window = int(sg_window)
        req.resample_dt = float(resample_dt)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", "", 0
        r = fut.result()
        return bool(r.success), str(r.message), str(r.output_path), int(r.rows)

    def traj_list(self) -> tuple[bool, str, list[str]]:
        cli = self.create_client(TrajList, f"{self.ns}/traj/list")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", []
        fut = cli.call_async(TrajList.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", []
        r = fut.result()
        return bool(r.success), str(r.message), [str(p) for p in r.paths]

    def traj_replay_start(
        self,
        path: str,
        rate_scale: float = 1.0,
        approach_first: bool = True,
        auto_record: bool = True,
    ) -> tuple[bool, str, str]:
        cli = self.create_client(TrajReplayStart, f"{self.ns}/traj/replay/start")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable", ""
        req = TrajReplayStart.Request()
        req.path = path
        req.rate_scale = float(rate_scale)
        req.approach_first = bool(approach_first)
        req.auto_record = bool(auto_record)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout", ""
        r = fut.result()
        return bool(r.success), str(r.message), str(getattr(r, "record_path", "") or "")

    def traj_replay_stop(self) -> tuple[bool, str]:
        cli = self.create_client(TrajReplayStop, f"{self.ns}/traj/replay/stop")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "unavailable"
        fut = cli.call_async(TrajReplayStop.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, "timeout"
        r = fut.result()
        return bool(r.success), str(r.message)


class RosSpinner:
    """Background spin for status callbacks; blocking calls spin on the node too."""

    def __init__(self, node: ArmRosClient) -> None:
        self.node = node
        self._stop = threading.Event()
        # Non-daemon: main must join so Ctrl+C cleanup waits for spin to exit.
        self._thread = threading.Thread(target=self._run, name="ros_spinner", daemon=False)
        self._lock = threading.Lock()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _run(self) -> None:
        from rclpy.executors import ExternalShutdownException

        while not self._stop.is_set():
            if not rclpy.ok():
                break
            try:
                with self._lock:
                    if self._stop.is_set() or not rclpy.ok():
                        break
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            except ExternalShutdownException:
                break
            except Exception:
                if self._stop.is_set() or not rclpy.ok():
                    break

    def call(self, fn, *args, **kwargs):
        with self._lock:
            return fn(*args, **kwargs)
