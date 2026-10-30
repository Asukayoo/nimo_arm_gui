"""Display-unit helpers (library SI ↔ HMI mm/deg, ZYX Euler). Not kinematics."""

from __future__ import annotations

import math
from typing import Sequence, Tuple

from geometry_msgs.msg import Pose, Quaternion


def rad2deg(x: float) -> float:
    return x * (180.0 / math.pi)


def deg2rad(x: float) -> float:
    return x * (math.pi / 180.0)


def m2mm(x: float) -> float:
    return x * 1000.0


def quat_to_zyx_deg(q: Quaternion) -> Tuple[float, float, float]:
    """Intrinsic ZYX (yaw-pitch-roll) in degrees. Matches Eigen eulerAngles(2,1,0)."""
    w, x, y, z = q.w, q.x, q.y, q.z
    # roll (X), pitch (Y), yaw (Z)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return rad2deg(yaw), rad2deg(pitch), rad2deg(roll)


def joint_deg(q: Sequence[float]) -> list[float]:
    return [rad2deg(float(v)) for v in q]


def cart_mm_zyx_phi(
    pose: Pose, phi_rad: float, phi_valid: bool
) -> list[tuple[str, float, str]]:
    """Rows: (label, value, unit) for X/Y/Z mm, Rz/Ry/Rx deg, Phi deg."""
    rz, ry, rx = quat_to_zyx_deg(pose.orientation)
    rows = [
        ("X", m2mm(pose.position.x), "mm"),
        ("Y", m2mm(pose.position.y), "mm"),
        ("Z", m2mm(pose.position.z), "mm"),
        ("Rz", rz, "deg"),
        ("Ry", ry, "deg"),
        ("Rx", rx, "deg"),
        ("Phi", rad2deg(phi_rad) if phi_valid else float("nan"), "deg"),
    ]
    return rows
