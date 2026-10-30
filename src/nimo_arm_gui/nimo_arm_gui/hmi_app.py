#!/usr/bin/env python3
"""Nimo dual-arm PyQt5 HMI."""

from __future__ import annotations

import signal
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from nimo_arm_gui.logging_setup import (
    QtLogHandler,
    attach_qt_handler,
    configure_logging,
    detach_qt_handlers,
    get_logger,
)
from nimo_arm_gui.ros_client import MODE_NAMES, OP_NAMES, ArmRosClient, RosSpinner
from nimo_arm_gui.si_display import cart_mm_zyx_phi, joint_deg

_log = get_logger(__name__)


def _log_result(
    action: str, ok: bool, msg: str = "", *, benign: bool = False
) -> None:
    """INFO on success, WARNING on failure; DEBUG when failure is expected/benign."""
    detail = (msg or "").strip()
    if ok:
        if detail and detail.lower() not in ("ok", "success"):
            _log.info("%s: %s", action, detail)
        else:
            _log.info("%s: ok", action)
        return
    if benign:
        _log.debug("%s failed: %s", action, detail or "unknown")
    else:
        _log.warning("%s failed: %s", action, detail or "unknown")

# Cart jog axes (display labels + units); order matches JogCart axis index 0..6.
_CART_ROWS = [
    ("X", "mm"),
    ("Y", "mm"),
    ("Z", "mm"),
    ("Rz", "deg"),
    ("Ry", "deg"),
    ("Rx", "deg"),
    ("Phi", "deg"),
]


def _fmt(v: float, digits: int = 2) -> str:
    if v != v:  # NaN
        return "—"
    return f"{v:.{digits}f}"


def _op_color(op: int) -> str:
    if op in (7, 8):  # Fault / EStop
        return "#c0392b"
    if op == 4:  # Moving
        return "#2980b9"
    if op == 3:  # Ready
        return "#27ae60"
    if op == 5:  # Guiding
        return "#8e44ad"
    if op in (1, 2, 6):
        return "#f39c12"
    return "#7f8c8d"


class ArmPanel(QWidget):
    def __init__(self, side: str, client: ArmRosClient, spinner: RosSpinner) -> None:
        super().__init__()
        self.side = side
        self.client = client
        self.spinner = spinner

        root = QVBoxLayout(self)
        self.state_lbl = QLabel("OpState: —")
        self.state_lbl.setFont(QFont("Sans", 12, QFont.Bold))
        self.detail_lbl = QLabel("mode / error / reject")
        self.detail_lbl.setWordWrap(True)
        root.addWidget(self.state_lbl)
        root.addWidget(self.detail_lbl)

        safe = QHBoxLayout()
        for name, label, danger in [
            ("enable", "Enable", False),
            ("disable", "Disable", False),
            ("stop", "Stop", False),
            ("estop", "E-STOP", True),
            ("clear_error", "ClearError", False),
            ("reset_estop", "ResetEStop", False),
        ]:
            btn = QPushButton(label)
            if danger:
                btn.setStyleSheet("background:#c0392b;color:white;font-weight:bold;")
            btn.clicked.connect(lambda _=False, n=name: self._trig(n))
            safe.addWidget(btn)
        root.addLayout(safe)

        mode_row = QHBoxLayout()
        self.mode_box = QComboBox()
        self.mode_box.addItems(MODE_NAMES)
        mode_btn = QPushButton("Set Mode")
        mode_btn.clicked.connect(self._set_mode)
        mode_row.addWidget(QLabel("ControlMode"))
        mode_row.addWidget(self.mode_box)
        mode_row.addWidget(mode_btn)
        root.addLayout(mode_row)

        brake_row = QHBoxLayout()
        hold = QPushButton("Brake Hold")
        release = QPushButton("Brake Release")
        hold.clicked.connect(lambda: self._brake(1))
        release.clicked.connect(lambda: self._brake(2))
        brake_row.addWidget(hold)
        brake_row.addWidget(release)
        root.addLayout(brake_row)

        # Joint / Cart jog pages: each row is - | ref/resp | +
        jog_tabs = QTabWidget()
        self._joint_val_lbls: list[QLabel] = []
        self._cart_val_lbls: list[QLabel] = []
        jog_tabs.addTab(self._build_jog_page("j", self._joint_val_lbls), "Joint")
        jog_tabs.addTab(self._build_jog_page("c", self._cart_val_lbls), "Cart")
        root.addWidget(jog_tabs)

        # MovJ: pick a saved joint waypoint for this arm
        mov = QGroupBox("MovJ (saved joint waypoint)")
        mrow = QHBoxLayout(mov)
        self.wp_combo = QComboBox()
        self.wp_combo.setMinimumWidth(160)
        self.wp_combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        refresh_wp = QPushButton("Refresh")
        refresh_wp.clicked.connect(self.refresh_waypoints)
        go = QPushButton("MovJ")
        go.clicked.connect(self._mov_j_waypoint)
        mrow.addWidget(self.wp_combo, stretch=1)
        mrow.addWidget(refresh_wp)
        mrow.addWidget(go)
        root.addWidget(mov)

        root.addStretch(1)

        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(30)  # keep under backend jog watchdog (HMI overlay ≥200 ms)
        self._jog_cmd = None
        self._jog_timer.timeout.connect(self._jog_tick)

        self.refresh_waypoints()

    def _build_jog_page(self, kind: str, val_lbls: list[QLabel]) -> QWidget:
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(4, 4, 4, 4)
        mono = QFont("Monospace", 10)

        grid.addWidget(QLabel(""), 0, 0)
        grid.addWidget(QLabel(""), 0, 1)
        hdr = QLabel("ref / resp")
        hdr.setAlignment(Qt.AlignCenter)
        grid.addWidget(hdr, 0, 2)
        grid.addWidget(QLabel(""), 0, 3)

        if kind == "j":
            labels = [f"J{i + 1} [deg]" for i in range(7)]
        else:
            labels = [f"{name} [{unit}]" for name, unit in _CART_ROWS]

        for i, name in enumerate(labels):
            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedWidth(40)
            plus.setFixedWidth(40)
            mid = QLabel("— / —")
            mid.setAlignment(Qt.AlignCenter)
            mid.setFont(mono)
            mid.setMinimumWidth(140)
            val_lbls.append(mid)

            if kind == "j":
                minus.pressed.connect(lambda ii=i: self._jog_j(ii, -1.0))
                plus.pressed.connect(lambda ii=i: self._jog_j(ii, 1.0))
            else:
                minus.pressed.connect(lambda ii=i: self._jog_c(ii, -1.0))
                plus.pressed.connect(lambda ii=i: self._jog_c(ii, 1.0))
            minus.released.connect(self._jog_release)
            plus.released.connect(self._jog_release)

            grid.addWidget(QLabel(name), i + 1, 0)
            grid.addWidget(minus, i + 1, 1)
            grid.addWidget(mid, i + 1, 2)
            grid.addWidget(plus, i + 1, 3)
        return page

    def refresh_waypoints(self) -> None:
        """Fill MovJ combo with waypoints that include this arm's joints."""
        cur = self.wp_combo.currentData()
        ok, msg, wps = self.spinner.call(self.client.waypoint_list)
        self.wp_combo.blockSignals(True)
        self.wp_combo.clear()
        if not ok:
            _log_result(f"{self.side}/waypoint_list", False, msg)
            self.wp_combo.blockSignals(False)
            return
        want_left = self.side == "left"
        for wp in wps:
            has = wp.has_left if want_left else wp.has_right
            if not has:
                continue
            self.wp_combo.addItem(wp.name, wp.name)
        # restore selection if still present
        if cur is not None:
            idx = self.wp_combo.findData(cur)
            if idx >= 0:
                self.wp_combo.setCurrentIndex(idx)
        self.wp_combo.blockSignals(False)

    def update_status(self, arm) -> None:
        op = int(arm.op_state)
        name = OP_NAMES[op] if 0 <= op < len(OP_NAMES) else str(op)
        self.state_lbl.setText(f"{self.side.upper()}  OpState: {name}")
        self.state_lbl.setStyleSheet(f"color:{_op_color(op)};")
        mode = MODE_NAMES[arm.control_mode] if arm.control_mode < len(MODE_NAMES) else str(arm.control_mode)
        self.detail_lbl.setText(
            f"mode={mode} settled={arm.control_mode_settled} kind={arm.motion_kind} "
            f"err={arm.error_message} reject={arm.reject_message} "
            f"alarm={arm.is_alarm}/{arm.alarm_source} vel_alarm={arm.is_joint_vel_alarm} "
            f"done={arm.motion_done}"
        )

        q_ref = joint_deg(arm.q_ref)
        q_resp = joint_deg(arm.q_resp)
        for i in range(7):
            self._joint_val_lbls[i].setText(f"{_fmt(q_ref[i])} / {_fmt(q_resp[i])}")

        ref_rows = cart_mm_zyx_phi(arm.tcp_ref, arm.phi_ref, arm.phi_ref_valid)
        resp_rows = cart_mm_zyx_phi(arm.tcp_resp, arm.phi_resp, arm.phi_resp_valid)
        for i in range(7):
            self._cart_val_lbls[i].setText(f"{_fmt(ref_rows[i][1])} / {_fmt(resp_rows[i][1])}")

    def _trig(self, name: str) -> None:
        before = ""
        if name in ("clear_error", "reset_estop", "estop"):
            st = self.client.status
            if st is not None:
                arm = st.left if self.side == "left" else st.right
                op = int(arm.op_state)
                op_name = OP_NAMES[op] if 0 <= op < len(OP_NAMES) else str(op)
                before = f" before={{op={op_name} err={arm.error_message}}}"
        ok, msg = self.spinner.call(self.client.call_arm_trigger, self.side, name)
        # disable while already off often returns bad_state — not actionable.
        benign = (not ok) and name == "disable" and "bad_state" in (msg or "").lower()
        _log_result(
            f"{self.side}/{name}{before}",
            ok,
            msg,
            benign=benign,
        )

    def _jog_release(self) -> None:
        """Jog release: send direction 0 (yaml-limited decelerate). Do NOT call Stop()."""
        self._jog_timer.stop()
        cmd = self._jog_cmd
        self._jog_cmd = None
        if not cmd:
            return
        kind, a, _ = cmd
        if kind == "j":
            self.client.publish_jog_j(self.side, a, 0.0)
        else:
            self.client.publish_jog_cart(self.side, a, 0.0)

    def _set_mode(self) -> None:
        ok, msg = self.spinner.call(
            self.client.set_control_mode, self.side, self.mode_box.currentIndex()
        )
        _log_result(f"{self.side}/set_control_mode", ok, msg)

    def _brake(self, cmd: int) -> None:
        ok, msg = self.spinner.call(self.client.set_brake, self.side, cmd)
        _log_result(f"{self.side}/set_brake", ok, msg)

    def _jog_j(self, joint: int, direction: float) -> None:
        self._jog_cmd = ("j", joint, float(direction))
        self._jog_timer.start()
        self._jog_tick()

    def _jog_c(self, axis: int, direction: float) -> None:
        self._jog_cmd = ("c", axis, float(direction))
        self._jog_timer.start()
        self._jog_tick()

    def _jog_tick(self) -> None:
        if not self._jog_cmd:
            return
        kind, a, direction = self._jog_cmd
        if kind == "j":
            self.client.publish_jog_j(self.side, a, direction)
        else:
            self.client.publish_jog_cart(self.side, a, direction)

    def _mov_j_waypoint(self) -> None:
        name = self.wp_combo.currentData()
        if not name:
            name = self.wp_combo.currentText().strip()
        if not name:
            _log.warning("%s/mov_j: no waypoint selected", self.side)
            return
        left = self.side == "left"
        right = self.side == "right"
        ok, msg = self.spinner.call(self.client.waypoint_goto, name, left, right)
        _log_result(f"{self.side}/mov_j {name}", ok, msg)


class MainWindow(QMainWindow):
    def __init__(self, client: ArmRosClient, spinner: RosSpinner) -> None:
        super().__init__()
        self.client = client
        self.spinner = spinner
        self._closing = False
        self._last_event_seq = 0
        self._prev_servoj = False
        self.setWindowTitle("Nimo Arm HMI")
        self.resize(1100, 800)

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        # --- Lifecycle ---
        life = QWidget()
        ll = QVBoxLayout(life)
        row = QHBoxLayout()
        for label, fn in [
            ("Init", self._init),
            ("Shutdown", self._shutdown),
            ("Go Home", self._home),
            ("Go Work", self._work),
        ]:
            b = QPushButton(label)
            b.clicked.connect(fn)
            row.addWidget(b)
        ll.addLayout(row)
        clear_row = QHBoxLayout()
        for label, side, name in [
            ("ClearError L", "left", "clear_error"),
            ("ClearError R", "right", "clear_error"),
            ("ResetEStop L", "left", "reset_estop"),
            ("ResetEStop R", "right", "reset_estop"),
        ]:
            b = QPushButton(label)
            b.clicked.connect(
                lambda _=False, s=side, n=name: self._arm_trig_logged(s, n)
            )
            clear_row.addWidget(b)
        ll.addLayout(clear_row)

        # Teleop motion forms (orthogonal to ControlMode; Pico ⟂ ServoJ)
        teleop_row = QHBoxLayout()
        self.teleop_chk = QCheckBox("Pico遥操作模式")
        self.teleop_chk.setToolTip(
            "set_teleop_motion(mode=Pico)；与 ControlMode 正交、与 ServoJ 遥操作互斥。"
            "开=允许 Pico 位姿流；仍需手柄左 grip 才会动。"
        )
        self.teleop_chk.toggled.connect(lambda checked: self._teleop_toggled(checked, "pico"))
        self.teleop_lamp = QLabel("●")
        self.teleop_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
        self.teleop_lamp.setToolTip("status.teleop_motion_enabled")
        self.servoj_teleop_chk = QCheckBox("ServoJ遥操作模式")
        self.servoj_teleop_chk.setToolTip(
            "set_teleop_motion(mode=ServoJ)；订 arm/*/target_joints → ServoJ(H=0.02s)。"
            "与 Pico 互斥；开启时禁止轨迹复现。"
            "勾选→rokea/drag_bridge/enable=true（Rokea enableDrag）；"
            "取消/自动退出→enable=false（disableDrag+下电）。"
            "须本机先起 rokea_drag_bridge（话题已含 Rokea→Marvin 轴 2/4/6 取反）。"
            "Marvin Fault/EStop/Alarm、对齐/ServoJ 失败、关节话题超时会自动退出本模式。"
        )
        self.servoj_teleop_chk.toggled.connect(
            lambda checked: self._teleop_toggled(checked, "servoj")
        )
        self.servoj_teleop_lamp = QLabel("●")
        self.servoj_teleop_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
        self.servoj_teleop_lamp.setToolTip("status.servoj_teleop_enabled")
        self.drag_bridge_lbl = QLabel("bridge:—")
        self.drag_bridge_lbl.setToolTip(
            "rokea/drag_bridge/status（本机 drag bridge 心跳；绿=拖拽在线）"
        )
        self.drag_bridge_lamp = QLabel("●")
        self.drag_bridge_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
        teleop_row.addWidget(self.teleop_chk)
        teleop_row.addWidget(self.teleop_lamp)
        teleop_row.addWidget(self.servoj_teleop_chk)
        teleop_row.addWidget(self.servoj_teleop_lamp)
        teleop_row.addWidget(self.drag_bridge_lbl)
        teleop_row.addWidget(self.drag_bridge_lamp)
        teleop_row.addStretch(1)
        ll.addLayout(teleop_row)
        self._teleop_syncing = False

        self.global_status = QLabel("initialized: —")
        self.global_status.setFont(QFont("Sans", 11, QFont.Bold))
        self.global_status.setWordWrap(True)
        ll.addWidget(self.global_status)
        self.fault_lbl = QLabel("fault/estop: —")
        self.fault_lbl.setWordWrap(True)
        self.fault_lbl.setStyleSheet("color:#7f8c8d;")
        ll.addWidget(self.fault_lbl)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        ll.addWidget(self.log_view)
        self._qt_log = QtLogHandler(self)
        self._qt_log.sig_record.connect(self.log_view.append)
        attach_qt_handler(self._qt_log)
        tabs.addTab(life, "Lifecycle")
        self._prev_op = {"left": -1, "right": -1}
        self._prev_err = {"left": "", "right": ""}

        # --- Arms ---
        arms = QWidget()
        al = QHBoxLayout(arms)
        self.left_panel = ArmPanel("left", client, spinner)
        self.right_panel = ArmPanel("right", client, spinner)
        al.addWidget(self.left_panel)
        al.addWidget(self.right_panel)
        tabs.addTab(arms, "Arms")

        # --- MovL / Servo ---
        adv = QWidget()
        adv_l = QVBoxLayout(adv)
        self.side_sel = QComboBox()
        self.side_sel.addItems(["left", "right"])
        adv_l.addWidget(self.side_sel)
        pose_box = QGroupBox("MovL / ServoP pose (m, quat wxyz)")
        pf = QFormLayout(pose_box)
        self.pose_spins = []
        for name in ["x", "y", "z", "qw", "qx", "qy", "qz"]:
            sp = QDoubleSpinBox()
            sp.setDecimals(4)
            sp.setRange(-10.0, 10.0)
            sp.setSingleStep(0.01)
            if name == "qw":
                sp.setValue(1.0)
            self.pose_spins.append(sp)
            pf.addRow(name, sp)
        prow = QHBoxLayout()
        movl = QPushButton("MovL")
        servop = QPushButton("ServoP once")
        movl.clicked.connect(self._mov_l)
        servop.clicked.connect(self._servo_p)
        prow.addWidget(movl)
        prow.addWidget(servop)
        pf.addRow(prow)
        adv_l.addWidget(pose_box)

        kd = QGroupBox("Joint impedance K/D")
        kf = QFormLayout(kd)
        self.k_edit = QLineEdit("22,10,10,15,9,15,7")
        self.d_edit = QLineEdit("0.3,0.3,0.3,0.3,0.3,0.3,0.3")
        kf.addRow("K", self.k_edit)
        kf.addRow("D", self.d_edit)
        kd_btn = QPushButton("SetJointImpKd")
        kd_btn.clicked.connect(self._set_kd)
        kf.addRow(kd_btn)
        adv_l.addWidget(kd)

        ckd = QGroupBox("Cartesian impedance K/D（默认 arm.yaml impedance.cart）")
        ckf = QFormLayout(ckd)
        self.cart_k_edit = QLineEdit("3000,3000,3000,100,100,100,20")
        self.cart_d_edit = QLineEdit("0.1,0.1,0.1,0.3,0.3,0.3,1.0")
        ckf.addRow("K", self.cart_k_edit)
        ckf.addRow("D", self.cart_d_edit)
        ckd_btn = QPushButton("SetCartImpKd")
        ckd_btn.clicked.connect(self._set_cart_kd)
        ckf.addRow(ckd_btn)
        adv_l.addWidget(ckd)
        adv_l.addStretch(1)
        tabs.addTab(adv, "Cartesian / Imp")

        # --- Waypoints ---
        wp = QWidget()
        wl = QVBoxLayout(wp)
        self.wp_list = QListWidget()
        wl.addWidget(self.wp_list)
        name_row = QHBoxLayout()
        self.wp_name = QLineEdit()
        self.wp_name.setPlaceholderText("waypoint name")
        self.wp_left = QCheckBox("Left")
        self.wp_left.setChecked(True)
        self.wp_right = QCheckBox("Right")
        self.wp_right.setChecked(True)
        name_row.addWidget(self.wp_name)
        name_row.addWidget(self.wp_left)
        name_row.addWidget(self.wp_right)
        wl.addLayout(name_row)
        btn_row = QHBoxLayout()
        for label, slot in [
            ("Refresh", self._wp_refresh),
            ("Record current", self._wp_save),
            ("Goto", self._wp_goto),
            ("Delete", self._wp_delete),
        ]:
            b = QPushButton(label)
            b.clicked.connect(slot)
            btn_row.addWidget(b)
        wl.addLayout(btn_row)
        tabs.addTab(wp, "Waypoints")

        # --- Record ---
        rec = QWidget()
        rl = QVBoxLayout(rec)
        self.rec_path = QLineEdit()
        self.rec_path.setPlaceholderText("empty = auto ~/.nimo_robot/records/")
        self.rec_period = QDoubleSpinBox()
        self.rec_period.setRange(0.002, 1.0)
        self.rec_period.setValue(0.02)
        self.rec_period.setSuffix(" s")
        self.rec_info = QLabel("idle")
        rf = QFormLayout()
        rf.addRow("path", self.rec_path)
        rf.addRow("sample", self.rec_period)
        rf.addRow("status", self.rec_info)
        rl.addLayout(rf)
        rr = QHBoxLayout()
        start = QPushButton("Start recording")
        stop = QPushButton("Stop recording")
        start.setStyleSheet("background:#27ae60;color:white;")
        stop.setStyleSheet("background:#c0392b;color:white;")
        start.clicked.connect(self._rec_start)
        stop.clicked.connect(self._rec_stop)
        rr.addWidget(start)
        rr.addWidget(stop)
        rl.addLayout(rr)
        rl.addStretch(1)
        tabs.addTab(rec, "Record")

        # --- Trajectory (executable .traj; NOT status Record CSV) ---
        traj = QWidget()
        tl = QVBoxLayout(traj)

        rec_box = QGroupBox("录制（停止后自动 SG 后处理 → *_smooth.traj）")
        t_rf = QFormLayout(rec_box)
        self.traj_rec_name = QLineEdit()
        self.traj_rec_name.setPlaceholderText("空=自动 traj_<时间>.traj；可填自定义名如 demo1")
        self.traj_rec_period = QDoubleSpinBox()
        self.traj_rec_period.setRange(0.002, 1.0)
        self.traj_rec_period.setValue(0.02)
        self.traj_rec_period.setSuffix(" s")
        self.traj_rec_info = QLabel("idle")
        t_rf.addRow("名称", self.traj_rec_name)
        t_rf.addRow("sample", self.traj_rec_period)
        t_rf.addRow("status", self.traj_rec_info)
        t_rr = QHBoxLayout()
        traj_start = QPushButton("Start traj record")
        traj_stop = QPushButton("Stop traj record")
        traj_start.setStyleSheet("background:#27ae60;color:white;")
        traj_stop.setStyleSheet("background:#c0392b;color:white;")
        traj_start.clicked.connect(self._traj_rec_start)
        traj_stop.clicked.connect(self._traj_rec_stop)
        t_rr.addWidget(traj_start)
        t_rr.addWidget(traj_stop)
        t_rf.addRow(t_rr)
        tl.addWidget(rec_box)

        rp_box = QGroupBox("复现（ServoJ；与 Pico/ServoJ 遥操作互斥）")
        rp_f = QFormLayout(rp_box)
        self.traj_rp_combo = QComboBox()
        self.traj_rp_combo.setMinimumWidth(360)
        traj_refresh = QPushButton("刷新")
        traj_refresh.clicked.connect(self._traj_refresh_list)
        traj_pick = QHBoxLayout()
        traj_pick.addWidget(self.traj_rp_combo, 1)
        traj_pick.addWidget(traj_refresh)
        self.traj_rp_rate = QDoubleSpinBox()
        self.traj_rp_rate.setRange(0.01, 5.0)
        self.traj_rp_rate.setValue(1.0)
        self.traj_rp_rate.setSingleStep(0.1)
        self.traj_rp_approach = QCheckBox("approach_first（先 MovJ 首点）")
        self.traj_rp_approach.setChecked(True)
        self.traj_rp_auto_rec = QCheckBox("自动记录状态 CSV（q/dq/TCP/tau）")
        self.traj_rp_auto_rec.setChecked(True)
        self.traj_rp_info = QLabel("idle")
        rp_f.addRow("轨迹", traj_pick)
        rp_f.addRow("rate_scale", self.traj_rp_rate)
        rp_f.addRow(self.traj_rp_approach)
        rp_f.addRow(self.traj_rp_auto_rec)
        rp_f.addRow("status", self.traj_rp_info)
        rp_row = QHBoxLayout()
        rp_start = QPushButton("Start replay")
        rp_stop = QPushButton("Stop replay")
        rp_start.setStyleSheet("background:#2980b9;color:white;")
        rp_stop.setStyleSheet("background:#c0392b;color:white;")
        rp_start.clicked.connect(self._traj_replay_start)
        rp_stop.clicked.connect(self._traj_replay_stop)
        rp_row.addWidget(rp_start)
        rp_row.addWidget(rp_stop)
        rp_f.addRow(rp_row)
        tl.addWidget(rp_box)

        self.traj_flags = QLabel("traj_recording=—  traj_replaying=—")
        tl.addWidget(self.traj_flags)
        tl.addStretch(1)
        tabs.addTab(traj, "轨迹")

        client.set_status_callback(self._on_status)
        self._wp_refresh()
        self._traj_refresh_list()

        self.ui_timer = QTimer(self)
        self.ui_timer.setInterval(100)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start()

    def _on_status(self, msg) -> None:
        # marshalled via timer to stay on GUI thread-ish; store only
        self._last_status = msg

    def _arm_summary(self, side: str, arm) -> str:
        op = int(arm.op_state)
        name = OP_NAMES[op] if 0 <= op < len(OP_NAMES) else str(op)
        return (
            f"{side}:{name} err={arm.error_message} "
            f"reject={arm.reject_message} alarm={arm.is_alarm}/{arm.alarm_source} "
            f"vel_alarm={arm.is_joint_vel_alarm}"
        )

    def _watch_faults(self, side: str, arm) -> None:
        op = int(arm.op_state)
        err = str(arm.error_message)
        prev = self._prev_op.get(side, -1)
        # rising edge into Fault / EStop
        if op == 7 and prev != 7:  # Fault
            _log.error(
                "FAULT %s: err=%s reject=%s alarm=%s vel_alarm=%s",
                side,
                err,
                arm.reject_message,
                arm.alarm_source,
                arm.is_joint_vel_alarm,
            )
        if op == 8 and prev != 8:  # EStop
            _log.error(
                "ESTOP %s: err=%s (soft EStop; recover with ResetEStop)",
                side,
                err,
            )
        # error code text changed while already faulted
        if op == 7 and err and err != self._prev_err.get(side, ""):
            if prev == 7:
                _log.error("FAULT %s err update: %s", side, err)
        self._prev_op[side] = op
        self._prev_err[side] = err

    def _arm_trig_logged(self, side: str, name: str) -> None:
        st = self.client.status
        before = ""
        if st is not None:
            arm = st.left if side == "left" else st.right
            before = (
                f" before={{op={OP_NAMES[arm.op_state] if arm.op_state < len(OP_NAMES) else arm.op_state}"
                f" err={arm.error_message}}}"
            )
        ok, msg = self.spinner.call(self.client.call_arm_trigger, side, name)
        _log_result(f"{side}/{name}{before}", ok, msg)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._closing = True
        if hasattr(self, "ui_timer"):
            self.ui_timer.stop()
        if hasattr(self, "left_panel") and hasattr(self.left_panel, "_jog_timer"):
            self.left_panel._jog_timer.stop()
        if hasattr(self, "right_panel") and hasattr(self.right_panel, "_jog_timer"):
            self.right_panel._jog_timer.stop()
        event.accept()

    def _refresh_ui(self) -> None:
        if self._closing or not rclpy.ok():
            return
        st = self.client.status
        if st is None:
            self.global_status.setText("initialized: — (no /nimo_arm/status)")
            self.fault_lbl.setText("fault/estop: —")
            self.teleop_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
            self.servoj_teleop_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
            self.drag_bridge_lamp.setStyleSheet("color:#7f8c8d;font-size:16px;")
            self.drag_bridge_lbl.setText("bridge:—")
            self.traj_flags.setText("traj_recording=—  traj_replaying=—")
            return
        teleop_on = bool(getattr(st, "teleop_motion_enabled", False))
        servoj_on = bool(getattr(st, "servoj_teleop_enabled", False))
        traj_rec = bool(getattr(st, "traj_recording", False))
        traj_rep = bool(getattr(st, "traj_replaying", False))
        bridge_ok = bool(self.client.drag_bridge_alive())
        # ServoJ abort may finish before HMI ever samples servoj=true (align fail).
        # Prefer last_event_seq edge; also catch falling edge while checkbox still on.
        seq = int(getattr(st, "last_event_seq", 0) or 0)
        event = str(getattr(st, "last_event", "") or "").strip()
        new_event = seq > self._last_event_seq
        if new_event and event.startswith("servoj_abort"):
            _log.warning("%s (seq=%s)", event, seq)
        elif (
            self._prev_servoj
            and not servoj_on
            and self.servoj_teleop_chk.isChecked()
        ):
            if event:
                _log.warning("%s (seq=%s)", event, seq)
            else:
                _log.warning("ServoJ teleop auto-off (no last_event on status)")
        if new_event:
            self._last_event_seq = seq
        self._prev_servoj = servoj_on
        self.global_status.setText(
            f"initialized={st.initialized}  run={st.run_thread_alive}  ts={st.ts:.4f}  "
            f"pico={teleop_on}  servoj={servoj_on}  bridge={bridge_ok}  "
            f"traj_rec={traj_rec}  traj_replay={traj_rep}"
        )
        self._sync_teleop_chk(teleop_on, servoj_on)
        self.teleop_lamp.setStyleSheet(
            f"color:{'#27ae60' if teleop_on else '#7f8c8d'};font-size:16px;"
        )
        self.servoj_teleop_lamp.setStyleSheet(
            f"color:{'#27ae60' if servoj_on else '#7f8c8d'};font-size:16px;"
        )
        self.drag_bridge_lamp.setStyleSheet(
            f"color:{'#27ae60' if bridge_ok else '#7f8c8d'};font-size:16px;"
        )
        self.drag_bridge_lbl.setText("bridge:on" if bridge_ok else "bridge:off")
        self.traj_flags.setText(
            f"traj_recording={traj_rec}  traj_replaying={traj_rep}"
        )
        if traj_rec and self.traj_rec_info.text().startswith("idle"):
            self.traj_rec_info.setText("recording…")
        if traj_rep:
            self.traj_rp_info.setText("replaying…")
        elif self.traj_rp_info.text() == "replaying…":
            self.traj_rp_info.setText("idle")
        self._watch_faults("left", st.left)
        self._watch_faults("right", st.right)
        self.fault_lbl.setText(
            f"{self._arm_summary('L', st.left)}  |  {self._arm_summary('R', st.right)}"
        )
        bad = int(st.left.op_state) in (7, 8) or int(st.right.op_state) in (7, 8)
        self.fault_lbl.setStyleSheet("color:#c0392b;" if bad else "color:#27ae60;")
        self.left_panel.update_status(st.left)
        self.right_panel.update_status(st.right)

    def _sync_teleop_chk(self, pico_on: bool, servoj_on: bool) -> None:
        self._teleop_syncing = True
        if self.teleop_chk.isChecked() != pico_on:
            self.teleop_chk.blockSignals(True)
            self.teleop_chk.setChecked(pico_on)
            self.teleop_chk.blockSignals(False)
        if self.servoj_teleop_chk.isChecked() != servoj_on:
            self.servoj_teleop_chk.blockSignals(True)
            self.servoj_teleop_chk.setChecked(servoj_on)
            self.servoj_teleop_chk.blockSignals(False)
        self._teleop_syncing = False

    def _teleop_toggled(self, checked: bool, which: str) -> None:
        if self._teleop_syncing:
            return
        mode = "servoj" if which == "servoj" else "pico"
        ok, msg = self.spinner.call(self.client.set_teleop_motion, checked, mode)
        _log_result(f"set_teleop_motion({mode}={checked})", ok, msg)
        if not ok:
            # revert the toggled checkbox; status timer will reconcile the other
            chk = self.servoj_teleop_chk if which == "servoj" else self.teleop_chk
            self._teleop_syncing = True
            chk.blockSignals(True)
            chk.setChecked(not checked)
            chk.blockSignals(False)
            self._teleop_syncing = False
            return
        # Mutual exclusion in UI immediately (backend also enforces).
        if checked:
            other = self.teleop_chk if which == "servoj" else self.servoj_teleop_chk
            if other.isChecked():
                self._teleop_syncing = True
                other.blockSignals(True)
                other.setChecked(False)
                other.blockSignals(False)
                self._teleop_syncing = False

    def _init(self) -> None:
        ok, msg = self.spinner.call(self.client.call_init)
        _log_result("init", ok, msg)
        if not ok:
            _log.warning("init FAILED — check backend / config: %s", msg)

    def _shutdown(self) -> None:
        ok, msg = self.spinner.call(self.client.call_shutdown)
        _log_result("shutdown", ok, msg)

    def _home(self) -> None:
        ok, msg = self.spinner.call(self.client.call_go_home)
        _log_result("go_home", ok, msg)

    def _work(self) -> None:
        ok, msg = self.spinner.call(self.client.call_go_work)
        _log_result("go_work", ok, msg)

    def _pose_msg(self) -> Pose:
        p = Pose()
        p.position.x = self.pose_spins[0].value()
        p.position.y = self.pose_spins[1].value()
        p.position.z = self.pose_spins[2].value()
        p.orientation.w = self.pose_spins[3].value()
        p.orientation.x = self.pose_spins[4].value()
        p.orientation.y = self.pose_spins[5].value()
        p.orientation.z = self.pose_spins[6].value()
        return p

    def _mov_l(self) -> None:
        side = self.side_sel.currentText()
        pose = self._pose_msg()
        zero = Pose()
        zero.orientation.w = 1.0
        left = side == "left"
        right = side == "right"
        ok, msg = self.spinner.call(
            self.client.send_mov_l,
            left,
            right,
            pose if left else zero,
            pose if right else zero,
        )
        _log_result("mov_l", ok, msg)

    def _servo_p(self) -> None:
        side = self.side_sel.currentText()
        self.client.publish_servo_p(side, self._pose_msg())
        _log.info("servo_p %s", side)

    def _set_kd(self) -> None:
        side = self.side_sel.currentText()
        try:
            k = [float(x) for x in self.k_edit.text().split(",")]
            d = [float(x) for x in self.d_edit.text().split(",")]
        except ValueError:
            _log.warning("bad K/D")
            return
        if len(k) != 7 or len(d) != 7:
            _log.warning("need 7 values")
            return
        ok, msg = self.spinner.call(self.client.set_joint_imp_kd, side, k, d)
        _log_result("set_joint_imp_kd", ok, msg)

    def _set_cart_kd(self) -> None:
        side = self.side_sel.currentText()
        try:
            k = [float(x) for x in self.cart_k_edit.text().split(",")]
            d = [float(x) for x in self.cart_d_edit.text().split(",")]
        except ValueError:
            _log.warning("bad cart K/D")
            return
        if len(k) != 7 or len(d) != 7:
            _log.warning("need 7 cart values")
            return
        ok, msg = self.spinner.call(self.client.set_cart_imp_kd, side, k, d)
        _log_result("set_cart_imp_kd", ok, msg)

    def _sync_arm_wp_combos(self) -> None:
        self.left_panel.refresh_waypoints()
        self.right_panel.refresh_waypoints()

    def _wp_refresh(self) -> None:
        ok, msg, wps = self.spinner.call(self.client.waypoint_list)
        self.wp_list.clear()
        if not ok:
            _log_result("waypoint_list", False, msg)
            self._sync_arm_wp_combos()
            return
        for wp in wps:
            flags = []
            if wp.has_left:
                flags.append("L")
            if wp.has_right:
                flags.append("R")
            self.wp_list.addItem(f"{wp.name} [{'+'.join(flags)}] {wp.note}")
        _log.debug("waypoints: %s", len(wps))
        self._sync_arm_wp_combos()

    def _wp_save(self) -> None:
        name = self.wp_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Waypoint", "Name required")
            return
        ok, msg = self.spinner.call(
            self.client.waypoint_save,
            name,
            self.wp_left.isChecked(),
            self.wp_right.isChecked(),
            "",
        )
        _log_result("waypoint_save", ok, msg)
        self._wp_refresh()

    def _selected_wp_name(self) -> Optional[str]:
        item = self.wp_list.currentItem()
        if item is None:
            name = self.wp_name.text().strip()
            return name or None
        return item.text().split(" ")[0]

    def _wp_goto(self) -> None:
        name = self._selected_wp_name()
        if not name:
            return
        ok, msg = self.spinner.call(
            self.client.waypoint_goto,
            name,
            self.wp_left.isChecked(),
            self.wp_right.isChecked(),
        )
        _log_result("waypoint_goto", ok, msg)

    def _wp_delete(self) -> None:
        name = self._selected_wp_name()
        if not name:
            return
        ok, msg = self.spinner.call(self.client.waypoint_delete, name)
        _log_result("waypoint_delete", ok, msg)
        self._wp_refresh()

    def _rec_start(self) -> None:
        ok, msg, path = self.spinner.call(
            self.client.record_start, self.rec_path.text().strip(), self.rec_period.value()
        )
        self.rec_info.setText(f"recording → {path}" if ok else msg)
        _log_result(f"record_start {path}", ok, msg)

    def _rec_stop(self) -> None:
        ok, msg, path, rows = self.spinner.call(self.client.record_stop)
        self.rec_info.setText(f"stopped rows={rows} path={path}" if ok else msg)
        _log_result(f"record_stop rows={rows}", ok, msg)

    def _traj_combo_select(self, path: str) -> None:
        if not path:
            return
        for i in range(self.traj_rp_combo.count()):
            if self.traj_rp_combo.itemData(i) == path:
                self.traj_rp_combo.setCurrentIndex(i)
                return
        # basename in UI, full path in userData
        name = path.rsplit("/", 1)[-1]
        self.traj_rp_combo.addItem(name, path)
        self.traj_rp_combo.setCurrentIndex(self.traj_rp_combo.count() - 1)

    def _traj_refresh_list(self) -> None:
        prev = self.traj_rp_combo.currentData()
        ok, msg, paths = self.spinner.call(self.client.traj_list)
        self.traj_rp_combo.clear()
        if not ok:
            _log_result("traj/list", False, msg)
            return
        # Prefer showing *_smooth.traj above raw twins when sorting already lexical
        for p in paths:
            self.traj_rp_combo.addItem(p.rsplit("/", 1)[-1], p)
        if prev:
            self._traj_combo_select(str(prev))
        elif self.traj_rp_combo.count() > 0:
            # default to last smooth if any
            pick = self.traj_rp_combo.count() - 1
            for i in range(self.traj_rp_combo.count()):
                name = self.traj_rp_combo.itemText(i)
                if name.endswith("_smooth.traj"):
                    pick = i
            self.traj_rp_combo.setCurrentIndex(pick)
        _log.debug("traj/list: %s", msg)

    def _traj_rec_start(self) -> None:
        name = self.traj_rec_name.text().strip()
        ok, msg, path = self.spinner.call(
            self.client.traj_record_start,
            name,
            self.traj_rec_period.value(),
        )
        self.traj_rec_info.setText(f"recording → {path}" if ok else msg)
        _log_result(f"traj/record/start {path}", ok, msg)

    def _traj_rec_stop(self) -> None:
        ok, msg, path, rows = self.spinner.call(self.client.traj_record_stop)
        if not ok:
            self.traj_rec_info.setText(msg)
            _log_result(f"traj/record/stop rows={rows}", False, msg)
            return
        self.traj_rec_info.setText(f"stopped rows={rows}; postprocessing…")
        _log_result(f"traj/record/stop rows={rows} {path}", True, msg)
        pok, pmsg, out, prows = self.spinner.call(
            self.client.traj_postprocess, path, "", 0, 0.0
        )
        if pok and out:
            self.traj_rec_info.setText(
                f"stopped rows={rows} → smooth rows={prows}"
            )
            self._traj_refresh_list()
            self._traj_combo_select(out)
            _log_result(f"traj/postprocess(auto) {out}", pok, pmsg)
        else:
            self.traj_rec_info.setText(
                f"stopped rows={rows}; postprocess failed: {pmsg}"
            )
            self._traj_refresh_list()
            if path:
                self._traj_combo_select(path)
            _log_result("traj/postprocess(auto)", pok, pmsg)

    def _traj_replay_start(self) -> None:
        path = self.traj_rp_combo.currentData()
        if not path:
            QMessageBox.warning(self, "轨迹", "请先选择轨迹（可点刷新）")
            return
        ok, msg, rec_path = self.spinner.call(
            self.client.traj_replay_start,
            str(path),
            self.traj_rp_rate.value(),
            self.traj_rp_approach.isChecked(),
            self.traj_rp_auto_rec.isChecked(),
        )
        if ok and rec_path:
            self.traj_rp_info.setText(f"replaying… record→ {rec_path}")
        else:
            self.traj_rp_info.setText("replaying…" if ok else msg)
        _log_result(f"traj/replay/start {path} record={rec_path}", ok, msg)

    def _traj_replay_stop(self) -> None:
        ok, msg = self.spinner.call(self.client.traj_replay_stop)
        self.traj_rp_info.setText("idle" if ok else msg)
        _log_result("traj/replay/stop", ok, msg)


def main(args=None) -> None:
    configure_logging()
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Override rclpy's SIGINT handler: quit Qt cleanly instead of KeyboardInterrupt
    # mid-callback (which left spin thread / double-shutdown mess).
    def _request_quit(*_args) -> None:
        app.quit()

    signal.signal(signal.SIGINT, _request_quit)
    signal.signal(signal.SIGTERM, _request_quit)
    # Qt blocks Python signal delivery unless the event loop wakes periodically.
    sig_pump = QTimer()
    sig_pump.start(50)
    sig_pump.timeout.connect(lambda: None)

    client = ArmRosClient()
    spinner = RosSpinner(client)
    spinner.start()
    win = MainWindow(client, spinner)
    win.show()

    code = 0
    try:
        code = app.exec_()
    finally:
        win._closing = True
        if hasattr(win, "ui_timer"):
            win.ui_timer.stop()
        detach_qt_handlers()
        spinner.stop()
        try:
            client.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)


if __name__ == "__main__":
    main()
