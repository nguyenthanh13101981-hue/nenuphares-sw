"""Formation coordinator skeleton (V1 minimal stub >= 2 Hz)."""

from nenuphar_msgs.msg import FormationRef

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import CMD_QOS


class FormationNode(MonitoredNode):
    """Minimal V1 interface scaffold for /formation/ref_state (>= 2 Hz)."""

    def __init__(self) -> None:
        super().__init__('formation')
        self.ref_pub = self.create_publisher(
            FormationRef, '/formation/ref_state', CMD_QOS
        )
        self.timer = self.create_timer(0.5, self._publish_ref)

    def _publish_ref(self) -> None:
        msg = FormationRef()
        msg.stamp = self.now_msg()
        msg.s_ref_m = 0.0
        msg.v_ref_mps = 0.0
        msg.role = FormationRef.ROLE_SINGLE_V1
        msg.gnss_clock_sync_ok = True
        self.ref_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(FormationNode, args)
