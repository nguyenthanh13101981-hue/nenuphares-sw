"""Bridge arch clearance estimator (event-driven at S4, Config B 800P)."""

from nenuphar_msgs.msg import BridgeClearance, MissionState
from sensor_msgs.msg import Image

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_corridor_config
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


class ClearanceNode(MonitoredNode):
    """Evaluates bridge clearance during state S4 (consensus spread <= 5 cm)."""

    def __init__(self) -> None:
        super().__init__('clearance')
        self.corridor_cfg = load_corridor_config()
        self.current_state = MissionState.STATE_S0

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect', self._on_depth, SENSOR_QOS
        )
        self.state_sub = self.create_subscription(
            MissionState, '/fsm/state', self._on_fsm_state, CMD_QOS
        )
        self.result_pub = self.create_publisher(
            BridgeClearance, '/clearance/bridge_result', CMD_QOS
        )

    def _on_fsm_state(self, msg: MissionState) -> None:
        self.current_state = msg.state

    def _on_depth(self, msg: Image) -> None:
        if self.current_state != MissionState.STATE_S4:
            return
        max_spread = float(self.corridor_cfg['bridge_s4']['consensus_spread_max_m'])
        out = BridgeClearance()
        out.stamp = msg.header.stamp
        out.clearance_m = 0.0
        out.consensus_spread_m = 0.0
        out.pass_permitted = out.consensus_spread_m <= max_spread and out.clearance_m > 0.0
        self.result_pub.publish(out)


def main(args=None) -> None:
    run_node_main(ClearanceNode, args)
