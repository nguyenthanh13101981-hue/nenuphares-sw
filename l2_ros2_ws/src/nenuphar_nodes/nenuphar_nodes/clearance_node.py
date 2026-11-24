"""Bridge arch clearance estimator (event-driven at S4, Config B 800P)."""

from nenuphar_msgs.msg import BridgeClearance, MissionState
from sensor_msgs.msg import Image

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.clearance_logic import evaluate_bridge_clearance
from nenuphar_nodes.config_loader import load_corridor_config, load_raft_profile
from nenuphar_nodes.fsm_engine import S4
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


class ClearanceNode(MonitoredNode):
    """Evaluates S4 bridge clearance >= raft_height + 10 cm, DB +/- 5 cm, spread <= 5 cm."""

    def __init__(self, **kwargs) -> None:
        super().__init__('clearance', **kwargs)
        self.declare_parameter('raft_profile', 'v1_70x50')
        self.declare_parameter('measured_clearance_m', 0.0)
        self.declare_parameter('consensus_spread_m', 0.0)
        self.declare_parameter('db_clearance_m', -1.0)

        self.corridor_cfg = load_corridor_config()
        profile_name = str(self.get_parameter('raft_profile').value)
        self.raft_profile = load_raft_profile(profile_name)
        self.raft_height_m = float(
            self.raft_profile['hull'].get('total_height_above_water_m', 0.45)
        )
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
        if self.current_state != S4:
            return
        bs4 = self.corridor_cfg['bridge_s4']
        margin_m = float(bs4.get('min_margin_above_raft_m', 0.10))
        max_spread = float(bs4['consensus_spread_max_m'])
        db_tol = float(bs4.get('db_tolerance_m', 0.05))

        clearance_m = float(self.get_parameter('measured_clearance_m').value)
        spread_m = float(self.get_parameter('consensus_spread_m').value)
        db_raw = float(self.get_parameter('db_clearance_m').value)
        db_val = db_raw if db_raw > 0.0 else None

        out = BridgeClearance()
        out.stamp = msg.header.stamp
        out.clearance_m = clearance_m
        out.consensus_spread_m = spread_m
        out.pass_permitted = evaluate_bridge_clearance(
            clearance_m=clearance_m,
            consensus_spread_m=spread_m,
            raft_height_m=self.raft_height_m,
            db_clearance_m=db_val,
            margin_m=margin_m,
            max_spread_m=max_spread,
            db_tolerance_m=db_tol,
        )
        self.result_pub.publish(out)


def main(args=None) -> None:
    run_node_main(ClearanceNode, args)
