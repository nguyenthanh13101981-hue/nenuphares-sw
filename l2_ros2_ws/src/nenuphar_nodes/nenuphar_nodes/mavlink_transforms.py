"""Coordinate frame transformations between ROS REP-103 (ENU/FLU) and MAVLink (NED/FRD)."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

# MAVLink frame constants (matching pymavlink mavutil.mavlink).
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12


@dataclass(frozen=True)
class MavlinkOdometryPayload:
    """Converted ODOMETRY payload ready for pymavlink odometry_send()."""

    time_usec: int
    frame_id: int
    child_frame_id: int
    x_ned: float
    y_ned: float
    z_ned: float
    q_wxyz: Tuple[float, float, float, float]
    vx_frd: float
    vy_frd: float
    vz_frd: float
    rollspeed_frd: float
    pitchspeed_frd: float
    yawspeed_frd: float
    pose_covariance: List[float]
    velocity_covariance: List[float]
    reset_counter: int


def _quat_mul_wxyz(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _normalize_quat_wxyz(
    q: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(c * c for c in q))
    if not math.isfinite(norm) or norm <= 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def _diag_cov_21(var_xyz: Tuple[float, float, float], var_rpy: Tuple[float, float, float]) -> List[float]:
    """Build finite 21-element upper-triangular 6x6 diagonal covariance vector."""
    cov = [0.0] * 21
    diag_vals = [var_xyz[0], var_xyz[1], var_xyz[2], var_rpy[0], var_rpy[1], var_rpy[2]]
    idx = 0
    for row in range(6):
        cov[idx] = float(diag_vals[row])
        idx += 6 - row
    return cov


def ros_enu_odom_to_mavlink_ned(
    pos_enu: Sequence[float],
    quat_xyzw_enu_flu: Sequence[float],
    vel_flu: Sequence[float],
    ang_vel_flu: Sequence[float],
    stamp_sec: int,
    stamp_nanosec: int,
    pose_covariance_enu: Optional[Sequence[float]] = None,
    vel_covariance_flu: Optional[Sequence[float]] = None,
    reset_counter: int = 0,
) -> MavlinkOdometryPayload:
    """Convert ROS REP-103 (ENU world, FLU body) Odometry into MAVLink (NED world, FRD body).

    Preserves valid 180-deg quaternions with w == 0.0 and uses the message timestamp.
    """
    x_e, y_e, z_e = float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2])
    qx, qy, qz, qw = (
        float(quat_xyzw_enu_flu[0]),
        float(quat_xyzw_enu_flu[1]),
        float(quat_xyzw_enu_flu[2]),
        float(quat_xyzw_enu_flu[3]),
    )
    q_enu_flu = _normalize_quat_wxyz((qw, qx, qy, qz))

    # q_NED<-ENU: 180 deg rotation around (1, 1, 0) / sqrt(2)
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    q_ned_from_enu = (0.0, -inv_sqrt2, -inv_sqrt2, 0.0)
    # q_FLU<-FRD: 180 deg rotation around +X
    q_flu_from_frd = (0.0, 1.0, 0.0, 0.0)
    q_ned_frd = _normalize_quat_wxyz(
        _quat_mul_wxyz(_quat_mul_wxyz(q_ned_from_enu, q_enu_flu), q_flu_from_frd)
    )

    vx, vy, vz = float(vel_flu[0]), float(vel_flu[1]), float(vel_flu[2])
    wx, wy, wz = float(ang_vel_flu[0]), float(ang_vel_flu[1]), float(ang_vel_flu[2])

    time_usec = max(0, int(stamp_sec) * 1_000_000 + int(stamp_nanosec) // 1000)

    if pose_covariance_enu is not None and len(pose_covariance_enu) == 36 and pose_covariance_enu[0] > 0.0:
        pose_cov = _diag_cov_21(
            (float(pose_covariance_enu[7]), float(pose_covariance_enu[0]), float(pose_covariance_enu[14])),
            (float(pose_covariance_enu[21]), float(pose_covariance_enu[28]), float(pose_covariance_enu[35])),
        )
    else:
        pose_cov = _diag_cov_21((0.02, 0.02, 0.05), (0.01, 0.01, 0.02))

    if vel_covariance_flu is not None and len(vel_covariance_flu) == 36 and vel_covariance_flu[0] > 0.0:
        vel_cov = _diag_cov_21(
            (float(vel_covariance_flu[0]), float(vel_covariance_flu[7]), float(vel_covariance_flu[14])),
            (float(vel_covariance_flu[21]), float(vel_covariance_flu[28]), float(vel_covariance_flu[35])),
        )
    else:
        vel_cov = _diag_cov_21((0.02, 0.02, 0.05), (0.01, 0.01, 0.02))

    return MavlinkOdometryPayload(
        time_usec=time_usec,
        frame_id=MAV_FRAME_LOCAL_NED,
        child_frame_id=MAV_FRAME_BODY_FRD,
        x_ned=y_e,
        y_ned=x_e,
        z_ned=-z_e,
        q_wxyz=q_ned_frd,
        vx_frd=vx,
        vy_frd=-vy,
        vz_frd=-vz,
        rollspeed_frd=wx,
        pitchspeed_frd=-wy,
        yawspeed_frd=-wz,
        pose_covariance=pose_cov,
        velocity_covariance=vel_cov,
        reset_counter=int(reset_counter) & 0xFF,
    )
