"""Mission FSM ROS 2 node wrapping MissionFsmEngine (S0-S12, 10 Hz + event-driven)."""

import time

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

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.fsm_engine import (
    FsmConfig,
    FsmInputs,
    MissionFsmEngine,
    PRIORITY_ORDER,
    resolve_fsm_state,
)
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

__all__ = ['MissionFsmNode', 'PRIORITY_ORDER', 'resolve_fsm_state']


class MissionFsmNode(MonitoredNode):
    """Mission state machine node (10 Hz state + immediate event publish, 5 Hz setpoints)."""

    def __init__(self, **kwargs) -> None:
        super().__init__('mission_fsm', **kwargs)
        self.declare_parameter('s3_timeout_s', 30.0)
        self.declare_parameter('s6_timeout_s', 20.0)

        cfg = FsmConfig(
            s3_timeout_s=float(self.get_parameter('s3_timeout_s').value),
            s6_timeout_s=float(self.get_parameter('s6_timeout_s').value),
        )
        self.engine = MissionFsmEngine(cfg)
        self.inputs = FsmInputs(
            now_s=time.monotonic(),
            usb3_camera_ok=False,
            all_nodes_alive=False,
        )
        self.last_output = self.engine.step(self.inputs)
        self._last_obstacle_stamp = None

        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_odom, SENSOR_QOS
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
        self.energy_sub = self.create_subscription(
            EnergyStatus, '/energy/status', self._on_energy, CMD_QOS
        )
        self.formation_sub = self.create_subscription(
            FormationRef, '/formation/ref_state', self._on_formation, CMD_QOS
        )
        self.health_sub = self.create_subscription(
            SystemHealth, '/system/health', self._on_health, CMD_QOS
        )

        self.state_pub = self.create_publisher(MissionState, '/fsm/state', CMD_QOS)
        self.setpoint_pub = self.create_publisher(
            GuidedSetpoint, '/fsm/cmd_setpoint', CMD_QOS
        )

        self.state_timer = self.create_timer(0.1, self._on_state_timer)
        self.setpoint_timer = self.create_timer(0.2, self._publish_setpoint)

    @property
    def state(self) -> int:
        return self.engine.state

    def _step_and_publish_on_change(self) -> None:
        prev = self.engine.state
        self.inputs.now_s = time.monotonic()
        self.last_output = self.engine.step(self.inputs)
        if self.engine.state != prev:
            self._publish_state()

    def _on_odom(self, _msg: Odometry) -> None:
        pass

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        self.inputs.obstacle_detected = len(msg.obstacles) > 0
        self._last_obstacle_stamp = msg.stamp
        self._step_and_publish_on_change()
        # Propagate source frame timestamp immediately for A.11 latency tracking.
        self._publish_setpoint(source_stamp=msg.stamp)

    def _on_clearance(self, msg: BridgeClearance) -> None:
        self.inputs.bridge_clearance_ready = True
        self.inputs.bridge_pass_permitted = bool(msg.pass_permitted)
        self._step_and_publish_on_change()

    def _on_corridor(self, msg: CorridorStatus) -> None:
        self.inputs.corridor_deviation_alert = bool(msg.deviation_alert)
        self._step_and_publish_on_change()

    def _on_energy(self, msg: EnergyStatus) -> None:
        self.inputs.energy_level = int(msg.level)
        self.inputs.bms_ble_connected = bool(msg.bms_ble_connected)
        self._step_and_publish_on_change()

    def _on_formation(self, msg: FormationRef) -> None:
        self.inputs.formation_hold_req = not bool(msg.gnss_clock_sync_ok)
        self._step_and_publish_on_change()

    def _on_health(self, msg: SystemHealth) -> None:
        self.inputs.usb3_camera_ok = bool(msg.usb3_camera_ok)
        self.inputs.all_nodes_alive = bool(msg.all_nodes_alive)
        self._step_and_publish_on_change()

    def _on_state_timer(self) -> None:
        self.inputs.now_s = time.monotonic()
        self.last_output = self.engine.step(self.inputs)
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
        sp.max_speed_mps = float(self.last_output.max_speed_mps)
        sp.hold_position = bool(self.last_output.hold_position)
        self.setpoint_pub.publish(sp)


def main(args=None) -> None:
    run_node_main(MissionFsmNode, args)
