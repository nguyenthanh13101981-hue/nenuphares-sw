"""MAVLink 2 bridge forwarding L2 heartbeat, VIO ODOMETRY, and EKF commands to L1."""

import time
from typing import Optional

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import (
    BridgeClearance,
    EnergyStatus,
    GuidedSetpoint,
    MissionState,
    SystemHealth,
)
from std_msgs.msg import UInt64

from nenuphar_nodes.base_node import (
    MAVLINK_HEARTBEAT_TX_TOPIC,
    MonitoredNode,
    run_node_main,
)
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

MAV_CMD_SET_EKF_SOURCE_SET = 42007


def parse_signing_key(hex_key: str) -> bytes:
    """Parse a 64-char hex string into a 32-byte MAVLink 2 signing key."""
    cleaned = hex_key.strip()
    key = bytes.fromhex(cleaned)
    if len(key) != 32:
        raise ValueError(f'MAVLink 2 signing key must be 32 bytes, got {len(key)}')
    return key


def configure_mavlink2_signing(mav_conn, key_bytes: bytes, link_id: int = 0) -> None:
    """Enable MAVLink 2 packet signing on a pymavlink connection."""
    if len(key_bytes) != 32:
        raise ValueError('MAVLink 2 signing key must be exactly 32 bytes')
    mav_conn.setup_signing(key_bytes, sign_outgoing=True, link_id=link_id)


class MavlinkBridgeNode(MonitoredNode):
    """Bridges /system/health, /vio/odometry, and bridge/EKF commands over MAVLink 2."""

    def __init__(self, **kwargs) -> None:
        super().__init__('mavlink_bridge', **kwargs)
        self.declare_parameter('mavlink_url', '')
        self.declare_parameter('mavlink_signing_key', '')

        self._mav = None
        self.signing_enabled = False
        mav_url = str(self.get_parameter('mavlink_url').value).strip()
        signing_hex = str(self.get_parameter('mavlink_signing_key').value).strip()
        if mav_url:
            from pymavlink import mavutil

            self._mav = mavutil.mavlink_connection(
                mav_url,
                source_system=1,
                source_component=191,
                dialect='ardupilotmega',
            )
            if signing_hex:
                configure_mavlink2_signing(self._mav, parse_signing_key(signing_hex))
                self.signing_enabled = True

        self.tx_heartbeat_count = 0
        self.tx_odom_count = 0
        self.last_ekf_source_set: Optional[int] = None
        self.last_health: Optional[SystemHealth] = None

        self.health_sub = self.create_subscription(
            SystemHealth, '/system/health', self._on_health, CMD_QOS
        )
        self.fsm_state_sub = self.create_subscription(
            MissionState, '/fsm/state', self._on_fsm_state, CMD_QOS
        )
        self.setpoint_sub = self.create_subscription(
            GuidedSetpoint, '/fsm/cmd_setpoint', self._on_setpoint, CMD_QOS
        )
        self.clearance_sub = self.create_subscription(
            BridgeClearance, '/clearance/bridge_result', self._on_clearance, CMD_QOS
        )
        self.energy_sub = self.create_subscription(
            EnergyStatus, '/energy/status', self._on_energy, CMD_QOS
        )
        self.vio_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_vio, SENSOR_QOS
        )

        self.hb_tx_pub = self.create_publisher(
            UInt64, MAVLINK_HEARTBEAT_TX_TOPIC, CMD_QOS
        )

    def send_ekf_source_set(self, source_set: int) -> None:
        """Send MAV_CMD_SET_EKF_SOURCE_SET (1=GNSS, 2=VIO ODOMETRY) to Pixhawk L1."""
        self.last_ekf_source_set = int(source_set)
        if self._mav is not None:
            self._mav.mav.command_long_send(
                self._mav.target_system or 1,
                self._mav.target_component or 1,
                MAV_CMD_SET_EKF_SOURCE_SET,
                0,
                float(source_set),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

    def _on_health(self, msg: SystemHealth) -> None:
        self.last_health = msg
        self.tx_heartbeat_count += 1

        if self._mav is not None:
            from pymavlink import mavutil

            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
                if msg.usb3_camera_ok
                else mavutil.mavlink.MAV_STATE_CRITICAL,
            )

        tx_msg = UInt64()
        tx_msg.data = self.tx_heartbeat_count
        self.hb_tx_pub.publish(tx_msg)

    def _on_fsm_state(self, _msg: MissionState) -> None:
        pass

    def _on_setpoint(self, _msg: GuidedSetpoint) -> None:
        pass

    def _on_clearance(self, _msg: BridgeClearance) -> None:
        pass

    def _on_energy(self, _msg: EnergyStatus) -> None:
        pass

    def _on_vio(self, msg: Odometry) -> None:
        self.tx_odom_count += 1
        if self._mav is not None:
            from pymavlink import mavutil

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            cov = [float('nan')] + [0.0] * 20
            self._mav.mav.odometry_send(
                int(time.time() * 1e6),
                mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                float(p.x),
                float(p.y),
                float(p.z),
                [float(q.w or 1.0), float(q.x), float(q.y), float(q.z)],
                float(v.x),
                float(v.y),
                float(v.z),
                float(w.x),
                float(w.y),
                float(w.z),
                cov,
                cov,
                0,
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
                100,
            )


def main(args=None) -> None:
    run_node_main(MavlinkBridgeNode, args)
