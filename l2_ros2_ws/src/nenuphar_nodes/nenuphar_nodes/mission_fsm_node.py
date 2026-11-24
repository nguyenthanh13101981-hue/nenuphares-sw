"""Mission FSM ROS 2 node wrapping MissionFsmEngine (CdCT A.8 S0-S12, ROS clock, /operator/cmd)."""

import math
from typing import Tuple

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import (
    BridgeClearance,
    CorridorStatus,
    EnergyStatus,
    FormationRef,
    GuidedSetpoint,
    MissionState,
    ObstacleArray,
    SystemHealth,
)
from std_msgs.msg import Bool, String

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.fsm_engine import (
    FsmConfig,
    FsmInputs,
    MissionFsmEngine,
    PRIORITY_ORDER,
    S4,
    S11,
    resolve_fsm_state,
)
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

__all__ = ['MissionFsmNode', 'PRIORITY_ORDER', 'resolve_fsm_state']


class MissionFsmNode(MonitoredNode):
    """Mission state machine node wired to /operator/cmd, /l1/ekf_odom, and perception/monitors."""

    def __init__(self, **kwargs) -> None:
        super().__init__('mission_fsm', **kwargs)
        self.declare_parameter('s3_centering_timeout_s', 30.0)
        self.declare_parameter('s6_rtk_timeout_s', 30.0)
        self.declare_parameter('s11_avoid_max_s', 10.0)
        self.declare_parameter('target_north_m', 50.0)
        self.declare_parameter('target_east_m', 0.0)
        self.declare_parameter('target_yaw_rad', 0.0)

        cfg = FsmConfig(
            s3_centering_timeout_s=float(self.get_parameter('s3_centering_timeout_s').value),
            s6_rtk_timeout_s=float(self.get_parameter('s6_rtk_timeout_s').value),
            s11_avoid_max_s=float(self.get_parameter('s11_avoid_max_s').value),
        )
        self.engine = MissionFsmEngine(cfg)
        self.inputs = FsmInputs(
            now_s=self._ros_now_s(),
            usb3_camera_ok=False,
            all_nodes_alive=False,
        )
        self.last_output = self.engine.step(self.inputs)

        # Navigation state in NED frame for real GuidedSetpoint generation (A6).
        self.target_ned: Tuple[float, float, float] = (
            float(self.get_parameter('target_north_m').value),
            float(self.get_parameter('target_east_m').value),
            float(self.get_parameter('target_yaw_rad').value),
        )
        self.current_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_obstacle_stamp = None

        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_odom, SENSOR_QOS
        )
        self.l1_odom_sub = self.create_subscription(
            Odometry, '/l1/ekf_odom', self._on_l1_odom, SENSOR_QOS
        )
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/perception/obstacles', self._on_obstacles, SENSOR_QOS
        )
        self.clearance_sub = self.create_subscription(
            BridgeClearance, '/clearance/bridge_result', self._on_clearance, CMD_QOS
        )
        self.corridor_sub = self.create_subscription(
            CorridorStatus, '/corridor/status', self._on_corridor, CMD_QOS
        )
        self.corridor_hold_sub = self.create_subscription(
            Bool, '/corridor/outside_z2_hold', self._on_corridor_hold, CMD_QOS
        )
        self.energy_sub = self.create_subscription(
            EnergyStatus, '/energy/status', self._on_energy, CMD_QOS
        )
        self.formation_sub = self.create_subscription(
            FormationRef, '/formation/ref_state', self._on_formation, CMD_QOS
        )
        self.health_sub = self.create_subscription(
            SystemHealth, '/system/health', self._on_health, CMD_QOS
        )
        self.operator_sub = self.create_subscription(
            String, '/operator/cmd', self._on_operator_cmd, CMD_QOS
        )

        self.state_pub = self.create_publisher(MissionState, '/fsm/state', CMD_QOS)
        self.setpoint_pub = self.create_publisher(
            GuidedSetpoint, '/fsm/cmd_setpoint', CMD_QOS
        )

        self.state_timer = self.create_timer(0.1, self._on_state_timer)
        self.setpoint_timer = self.create_timer(0.2, self._publish_setpoint)

    def _ros_now_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    @property
    def state(self) -> int:
        return self.engine.state

    def _step_and_publish_on_change(self) -> None:
        prev = self.engine.state
        self.inputs.now_s = self._ros_now_s()
        self.last_output = self.engine.step(self.inputs)
        if prev == S4 and self.engine.state != S4:
            # B1: Clear cached bridge clearance when leaving S4 so stale data is never reused.
            self.inputs.bridge_clearance_ready = False
            self.inputs.bridge_pass_permitted = False
        if self.engine.state != prev:
            self._publish_state()

    def _on_odom(self, msg: Odometry) -> None:
        # ENU -> NED for internal tracking if L1 odom not yet streaming.
        p = msg.pose.pose.position
        self.current_ned = (float(p.y), float(p.x), self.current_ned[2])

    def _on_l1_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_ned = (float(p.x), float(p.y), yaw)

    def _on_operator_cmd(self, msg: String) -> None:
        raw = msg.data.strip()
        cmd = raw.upper()
        if cmd == 'START':
            self.inputs.start_mission = True
            self.inputs.hold_requested = False
            self.inputs.reset_to_s0 = False
        elif cmd == 'HOLD':
            self.inputs.hold_requested = True
        elif cmd == 'RESUME':
            self.inputs.hold_requested = False
        elif cmd == 'RTB':
            self.inputs.rtb_requested = True
        elif cmd == 'ESTOP':
            self.inputs.fault_detected_s = self._ros_now_s()
            self.inputs.emergency_stop_cmd = True
        elif cmd == 'RESET':
            self.inputs.emergency_stop_cmd = False
            self.inputs.tier3_critical_fault = False
            self.inputs.rtb_requested = False
            self.inputs.hold_requested = False
            self.inputs.docked_at_base = False
            self.inputs.bridge_detected = False
            self.inputs.at_bridge_centering = False
            self.inputs.bridge_centered = False
            self.inputs.bridge_clearance_ready = False
            self.inputs.bridge_pass_permitted = False
            self.inputs.bridge_exited = False
            self.inputs.gnss_reacquired = False
            self.inputs.reset_to_s0 = True
        elif cmd.startswith('APPROACH_BRIDGE'):
            parts = raw.split(':', 1)
            self.inputs.bridge_id = parts[1].strip() if len(parts) > 1 else 'bridge_01'
            self.inputs.bridge_detected = True
        elif cmd == 'AT_CENTERING':
            self.inputs.at_bridge_centering = True
        elif cmd == 'AT_MEASURE':
            self.inputs.bridge_centered = True
        elif cmd == 'BRIDGE_EXITED':
            self.inputs.bridge_exited = True
        elif cmd == 'GNSS_REACQUIRED':
            self.inputs.gnss_reacquired = True
            self.inputs.bridge_detected = False
            self.inputs.at_bridge_centering = False
            self.inputs.bridge_centered = False
            self.inputs.bridge_exited = False
        elif cmd == 'DOCKED':
            self.inputs.docked_at_base = True
        elif cmd.startswith('SET_WAYPOINT:'):
            coords = raw.split(':', 1)[1].split(',')
            if len(coords) >= 2:
                n_m = float(coords[0])
                e_m = float(coords[1])
                y_r = float(coords[2]) if len(coords) >= 3 else self.target_ned[2]
                self.target_ned = (n_m, e_m, y_r)
        self._step_and_publish_on_change()

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        self.inputs.obstacle_detected = len(msg.obstacles) > 0
        self._last_obstacle_stamp = msg.stamp
        self._step_and_publish_on_change()
        self._publish_setpoint(source_stamp=msg.stamp)

    def _on_clearance(self, msg: BridgeClearance) -> None:
        self.inputs.bridge_clearance_ready = True
        self.inputs.bridge_pass_permitted = bool(msg.pass_permitted)
        self._step_and_publish_on_change()

    def _on_corridor(self, msg: CorridorStatus) -> None:
        self.inputs.corridor_ddkc_alert = bool(msg.deviation_alert)
        self._step_and_publish_on_change()

    def _on_corridor_hold(self, msg: Bool) -> None:
        self.inputs.outside_z2_hold = bool(msg.data)
        self._step_and_publish_on_change()

    def _on_energy(self, msg: EnergyStatus) -> None:
        self.inputs.energy_level = int(msg.level)
        self.inputs.bms_ble_connected = bool(msg.bms_ble_connected)
        self._step_and_publish_on_change()

    def _on_formation(self, msg: FormationRef) -> None:
        self.inputs.hold_requested = (not bool(msg.gnss_clock_sync_ok)) or self.inputs.hold_requested
        self._step_and_publish_on_change()

    def _on_health(self, msg: SystemHealth) -> None:
        self.inputs.usb3_camera_ok = bool(msg.usb3_camera_ok)
        if not msg.all_nodes_alive and self.inputs.all_nodes_alive:
            self.inputs.fault_detected_s = self._ros_now_s()
        self.inputs.all_nodes_alive = bool(msg.all_nodes_alive)
        self._step_and_publish_on_change()

    def _on_state_timer(self) -> None:
        self._step_and_publish_on_change()
        self._publish_state()

    def _publish_state(self) -> None:
        msg = MissionState()
        msg.stamp = self.now_msg()
        msg.state = self.last_output.state
        msg.fault_tier = self.last_output.fault_tier
        msg.bridge_permit = self.last_output.bridge_permit
        msg.hold_all_req = self.last_output.hold_all_req
        self.state_pub.publish(msg)

    def _publish_setpoint(self, source_stamp=None) -> None:
        sp = GuidedSetpoint()
        sp.stamp = source_stamp if source_stamp is not None else self.now_msg()
        if self.last_output.hold_position:
            sp.north_m = float(self.current_ned[0])
            sp.east_m = float(self.current_ned[1])
            sp.yaw_rad = float(self.current_ned[2])
        else:
            # Apply controlled lateral avoidance offset within Z2 (+/- 1.25 m) during S11.
            lateral_offset = 0.80 if self.last_output.state == S11 else 0.0
            sp.north_m = float(self.target_ned[0])
            sp.east_m = float(self.target_ned[1] + lateral_offset)
            sp.yaw_rad = float(self.target_ned[2])
        sp.max_speed_mps = float(self.last_output.max_speed_mps)
        sp.hold_position = bool(self.last_output.hold_position)
        self.setpoint_pub.publish(sp)


def main(args=None) -> None:
    run_node_main(MissionFsmNode, args)
