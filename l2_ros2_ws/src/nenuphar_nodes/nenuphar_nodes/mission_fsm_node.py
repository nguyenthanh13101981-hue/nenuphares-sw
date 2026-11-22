"""Mission FSM S0-S12 with hard priority order S9 > S8 > S12 > S7 > S11 > S10 > S1."""

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

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

# Hard priority order from PROJECT_RULES.md §3 rule 5.
PRIORITY_ORDER: Tuple[int, ...] = (
    MissionState.STATE_S9,
    MissionState.STATE_S8,
    MissionState.STATE_S12,
    MissionState.STATE_S7,
    MissionState.STATE_S11,
    MissionState.STATE_S10,
    MissionState.STATE_S1,
)


def resolve_fsm_state(
    current_state: int,
    usb3_camera_ok: bool,
    requests: Tuple[int, ...] = (),
) -> int:
    """Resolve next state honoring S0 USB-3 gate and hard priority S9>S8>S12>S7>S11>S10>S1."""
    if not usb3_camera_ok and current_state == MissionState.STATE_S0:
        # Reject leaving S0 unless emergency/stop state is requested.
        emergency = (
            MissionState.STATE_S9,
            MissionState.STATE_S8,
            MissionState.STATE_S12,
        )
        for state in emergency:
            if state in requests:
                return state
        return MissionState.STATE_S0

    for state in PRIORITY_ORDER:
        if state in requests:
            return state
    return current_state


class MissionFsmNode(MonitoredNode):
    """Mission state machine node (10 Hz state, 5-10 Hz Guided setpoints)."""

    def __init__(self) -> None:
        super().__init__('mission_fsm')
        self.state = MissionState.STATE_S0
        self.fault_tier = 0
        self.bridge_permit = False
        self.hold_all_req = False
        self.usb3_camera_ok = False
        self.latest_energy_level = EnergyStatus.LEVEL_OK
        self.latest_corridor_alert = False

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

        self.state_timer = self.create_timer(0.1, self._publish_state)
        self.setpoint_timer = self.create_timer(0.2, self._publish_setpoint)

    def _on_odom(self, _msg: Odometry) -> None:
        pass

    def _on_obstacles(self, _msg: ObstacleArray) -> None:
        pass

    def _on_clearance(self, msg: BridgeClearance) -> None:
        self.bridge_permit = bool(msg.pass_permitted)

    def _on_corridor(self, msg: CorridorStatus) -> None:
        self.latest_corridor_alert = bool(msg.deviation_alert)

    def _on_energy(self, msg: EnergyStatus) -> None:
        self.latest_energy_level = int(msg.level)

    def _on_formation(self, _msg: FormationRef) -> None:
        pass

    def _on_health(self, msg: SystemHealth) -> None:
        self.usb3_camera_ok = bool(msg.usb3_camera_ok)
        self._step_fsm()

    def _step_fsm(self) -> None:
        reqs = []
        if self.latest_energy_level == EnergyStatus.LEVEL_STOP:
            reqs.append(MissionState.STATE_S9)
            self.fault_tier = 3
        elif self.latest_energy_level == EnergyStatus.LEVEL_RTB:
            reqs.append(MissionState.STATE_S10)
        elif self.latest_corridor_alert:
            reqs.append(MissionState.STATE_S12)
            self.fault_tier = max(self.fault_tier, 2)
        self.state = resolve_fsm_state(
            self.state, self.usb3_camera_ok, tuple(reqs)
        )

    def _publish_state(self) -> None:
        msg = MissionState()
        msg.stamp = self.now_msg()
        msg.state = self.state
        msg.fault_tier = self.fault_tier
        msg.bridge_permit = self.bridge_permit
        msg.hold_all_req = self.hold_all_req
        self.state_pub.publish(msg)

    def _publish_setpoint(self) -> None:
        sp = GuidedSetpoint()
        sp.stamp = self.now_msg()
        sp.hold_position = self.state in (
            MissionState.STATE_S0,
            MissionState.STATE_S8,
            MissionState.STATE_S9,
            MissionState.STATE_S12,
        )
        self.setpoint_pub.publish(sp)


def main(args=None) -> None:
    run_node_main(MissionFsmNode, args)
