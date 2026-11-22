"""Low-priority blackbox rosbag2 recorder skeleton (1 Hz normal, >= 10 Hz event/S4)."""

from nenuphar_msgs.msg import MissionState, SystemHealth

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import CMD_QOS


class BlackboxNode(MonitoredNode):
    """Tracks logging rate based on mission state and fault tier."""

    def __init__(self) -> None:
        super().__init__('blackbox')
        self.record_rate_hz = 1.0
        self.last_state = MissionState.STATE_S0

        self.fsm_sub = self.create_subscription(
            MissionState, '/fsm/state', self._on_fsm_state, CMD_QOS
        )
        self.health_sub = self.create_subscription(
            SystemHealth, '/system/health', self._on_health, CMD_QOS
        )

    def _on_fsm_state(self, msg: MissionState) -> None:
        self.last_state = msg.state
        high_rate = msg.state == MissionState.STATE_S4 or msg.fault_tier >= 2
        self.record_rate_hz = 10.0 if high_rate else 1.0

    def _on_health(self, _msg: SystemHealth) -> None:
        pass


def main(args=None) -> None:
    run_node_main(BlackboxNode, args)
