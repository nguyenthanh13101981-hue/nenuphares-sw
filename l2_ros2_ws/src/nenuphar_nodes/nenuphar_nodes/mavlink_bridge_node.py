"""MAVLink 2 bidirectional bridge forwarding L2 <-> L1 and keepalive/BRIDGE_ZONE to C8."""

import time
from typing import List, Optional

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import (
    BridgeClearance,
    EnergyStatus,
    GuidedSetpoint,
    MissionState,
    SystemHealth,
)
from std_msgs.msg import Float64, String, UInt64

from nenuphar_nodes.base_node import (
    MAVLINK_HEARTBEAT_TX_TOPIC,
    MonitoredNode,
    run_node_main,
)
from nenuphar_nodes.fsm_engine import S4, S5, S7, S8, S9
from nenuphar_nodes.mavlink_transforms import ros_enu_odom_to_mavlink_ned
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_SET_EKF_SOURCE_SET = 42007
E2E_LATENCY_TOPIC = '/mavlink/e2e_latency_ms'
C8_TX_TOPIC = '/c8/tx_signals'


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
    """Bidirectional MAVLink 2 bridge (L2 <-> L1) + C8 keepalive/BRIDGE_ZONE emitter."""

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
        self.tx_setpoint_count = 0
        self.tx_c8_keepalive_count = 0
        self.tx_c8_bridge_zone_count = 0
        self.e2e_latencies_ms: List[float] = []
        self.last_ekf_source_set: Optional[int] = None
        self.last_bridge_permit: Optional[bool] = None
        self.last_bridge_zone: Optional[bool] = None
        self.last_speed_cmd: Optional[float] = None
        self.last_health: Optional[SystemHealth] = None
        self.l1_mode: Optional[int] = None
        self._odom_reset_counter = 0

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
        self.latency_pub = self.create_publisher(
            Float64, E2E_LATENCY_TOPIC, SENSOR_QOS
        )
        self.c8_tx_pub = self.create_publisher(String, C8_TX_TOPIC, CMD_QOS)
        self.l1_shunt_pub = self.create_publisher(
            EnergyStatus, '/l1/battery_shunt', CMD_QOS
        )
        self.l1_odom_pub = self.create_publisher(
            Odometry, '/l1/ekf_odom', SENSOR_QOS
        )
        self.operator_pub = self.create_publisher(String, '/operator/cmd', CMD_QOS)

        self._rx_timer = self.create_timer(0.05, self._poll_mavlink_rx)

    def enable_signing(self, signing_hex: str, link_id: int = 0) -> None:
        """Activate strict MAVLink 2 signing on an established connection."""
        if self._mav is None:
            raise RuntimeError('MAVLink connection is not initialized')
        configure_mavlink2_signing(self._mav, parse_signing_key(signing_hex), link_id=link_id)
        self.signing_enabled = True

    def _set_l1_param(self, name: str, value: float) -> None:
        if self._mav is not None:
            from pymavlink import mavutil

            self._mav.mav.param_set_send(
                self._mav.target_system or 1,
                self._mav.target_component or 1,
                name.encode('ascii'),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
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

    def send_speed_limit(self, max_speed_mps: float) -> None:
        """Send MAV_CMD_DO_CHANGE_SPEED to Pixhawk L1 when max_speed_mps updates."""
        self.last_speed_cmd = float(max_speed_mps)
        if self._mav is not None:
            self._mav.mav.command_long_send(
                self._mav.target_system or 1,
                self._mav.target_component or 1,
                MAV_CMD_DO_CHANGE_SPEED,
                0,
                1.0,  # ground speed
                float(max_speed_mps),
                -1.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

    def _emit_c8_signal(self, signal_name: str) -> None:
        msg = String()
        msg.data = signal_name
        self.c8_tx_pub.publish(msg)

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

        # Forward 1 Hz keepalive to C8 only when system health is healthy.
        if msg.all_nodes_alive:
            self.tx_c8_keepalive_count += 1
            self._emit_c8_signal('KEEPALIVE')

    def _on_fsm_state(self, msg: MissionState) -> None:
        # 1. Switch EKF source dynamically (2 = VIO under bridge S5/S8, 1 = GNSS elsewhere).
        desired_ekf = 2 if msg.state in (S5, S8) else 1
        if self.last_ekf_source_set != desired_ekf:
            self.send_ekf_source_set(desired_ekf)

        # 2. Sync BRIDGE_ZONE (SCR_USER5) & BRIDGE_PERMIT (SCR_USER4) to L1 Lua + C8.
        desired_bz = msg.state in (S4, S5, S8)
        desired_permit = bool(msg.bridge_permit)
        if self.last_bridge_permit != desired_permit:
            self.last_bridge_permit = desired_permit
            self._set_l1_param('SCR_USER4', 1.0 if desired_permit else 0.0)
        if self.last_bridge_zone != desired_bz:
            self.last_bridge_zone = desired_bz
            self._set_l1_param('SCR_USER5', 1.0 if desired_bz else 0.0)
        if desired_bz:
            self.tx_c8_bridge_zone_count += 1
            self._emit_c8_signal('BRIDGE_ZONE')

        # 3. Command HOLD mode on Pixhawk for S7/S9.
        if self._mav is not None and msg.state in (S7, S9):
            self._mav.set_mode(4)

    def _on_setpoint(self, msg: GuidedSetpoint) -> None:
        self.tx_setpoint_count += 1
        if msg.stamp.sec > 0 or msg.stamp.nanosec > 0:
            now_ns = self.get_clock().now().nanoseconds
            src_ns = int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec)
            latency_ms = max(0.0, (now_ns - src_ns) / 1e6)
            self.e2e_latencies_ms.append(latency_ms)
            if len(self.e2e_latencies_ms) > 2000:
                self.e2e_latencies_ms.pop(0)
            lat_msg = Float64()
            lat_msg.data = latency_ms
            self.latency_pub.publish(lat_msg)

        if self.last_speed_cmd is None or abs(self.last_speed_cmd - float(msg.max_speed_mps)) > 1e-3:
            self.send_speed_limit(float(msg.max_speed_mps))

        if self._mav is not None:
            from pymavlink import mavutil

            # Type mask: position + yaw active (ignore velocity/accel/yaw_rate).
            type_mask = 0x09F8 if not msg.hold_position else 0x0DF8
            self._mav.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                self._mav.target_system or 1,
                self._mav.target_component or 1,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(msg.north_m),
                float(msg.east_m),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                float(msg.yaw_rad),
                0.0,
            )

    def _on_clearance(self, _msg: BridgeClearance) -> None:
        pass

    def _on_energy(self, _msg: EnergyStatus) -> None:
        pass

    def _on_vio(self, msg: Odometry) -> None:
        self.tx_odom_count += 1
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        payload = ros_enu_odom_to_mavlink_ned(
            pos_enu=(p.x, p.y, p.z),
            quat_xyzw_enu_flu=(q.x, q.y, q.z, q.w),
            vel_flu=(v.x, v.y, v.z),
            ang_vel_flu=(w.x, w.y, w.z),
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            pose_covariance_enu=list(msg.pose.covariance),
            vel_covariance_flu=list(msg.twist.covariance),
            reset_counter=self._odom_reset_counter,
        )
        if self._mav is not None:
            from pymavlink import mavutil

            self._mav.mav.odometry_send(
                payload.time_usec,
                payload.frame_id,
                payload.child_frame_id,
                payload.x_ned,
                payload.y_ned,
                payload.z_ned,
                list(payload.q_wxyz),
                payload.vx_frd,
                payload.vy_frd,
                payload.vz_frd,
                payload.rollspeed_frd,
                payload.pitchspeed_frd,
                payload.yawspeed_frd,
                payload.pose_covariance,
                payload.velocity_covariance,
                payload.reset_counter,
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
                100,
            )

    def _handle_mavlink_msg(self, msg) -> None:
        """Process inbound L1 telemetry & GCS operator commands (testable without socket)."""
        mtype = msg.get_type()
        if mtype == 'HEARTBEAT' and msg.get_srcComponent() == 1:
            self.l1_mode = int(msg.custom_mode)
        elif mtype == 'BATTERY_STATUS':
            volts_mv = [v for v in getattr(msg, 'voltages', []) if 0 < v < 65535]
            volt_v = (sum(volts_mv) / 1000.0) if volts_mv else 0.0
            curr_ca = float(getattr(msg, 'current_battery', -1))
            curr_a = (curr_ca / 100.0) if curr_ca >= 0 else 0.0
            soc = float(getattr(msg, 'battery_remaining', -1))
            out = EnergyStatus()
            out.stamp = self.now_msg()
            out.voltage_v = volt_v
            out.current_a = curr_a
            out.soc_pct = max(0.0, soc) if soc >= 0 else 100.0
            self.l1_shunt_pub.publish(out)
        elif mtype == 'LOCAL_POSITION_NED':
            odom = Odometry()
            odom.header.stamp = self.now_msg()
            odom.header.frame_id = 'ned'
            odom.pose.pose.position.x = float(msg.x)
            odom.pose.pose.position.y = float(msg.y)
            odom.pose.pose.position.z = float(msg.z)
            self.l1_odom_pub.publish(odom)
        elif mtype == 'COMMAND_LONG':
            # Map user MAV_CMD_USER_1..5 (31010..31014) from QGC to /operator/cmd
            cmd_id = int(getattr(msg, 'command', 0))
            mapping = {
                31010: 'START',
                31011: 'HOLD',
                31012: 'RTB',
                31013: 'RESET',
                31014: 'ESTOP',
            }
            if cmd_id in mapping:
                op = String()
                op.data = mapping[cmd_id]
                self.operator_pub.publish(op)

    def _poll_mavlink_rx(self) -> None:
        if self._mav is None:
            return
        for _ in range(16):
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            self._handle_mavlink_msg(msg)


def main(args=None) -> None:
    run_node_main(MavlinkBridgeNode, args)
