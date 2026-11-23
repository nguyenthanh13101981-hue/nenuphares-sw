"""Formation coordinator V1 minimal interface (>= 2 Hz, GNSS clock sync <= 10 ms)."""

from nenuphar_msgs.msg import FormationRef

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import CMD_QOS


class FormationNode(MonitoredNode):
    """Minimal V1 single-raft formation scaffold publishing /formation/ref_state >= 2 Hz."""

    def __init__(self, **kwargs) -> None:
        super().__init__('formation', **kwargs)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('nominal_speed_mps', 0.0)
        self.declare_parameter('gnss_clock_offset_ms', 0.0)
        self.declare_parameter('gnss_clock_max_offset_ms', 10.0)

        self.s_ref_m = 0.0
        rate_hz = max(2.0, float(self.get_parameter('rate_hz').value))
        self.period_s = 1.0 / rate_hz

        self.ref_pub = self.create_publisher(
            FormationRef, '/formation/ref_state', CMD_QOS
        )
        self.timer = self.create_timer(self.period_s, self._publish_ref)

    def _publish_ref(self) -> None:
        v_ref = float(self.get_parameter('nominal_speed_mps').value)
        offset_ms = abs(float(self.get_parameter('gnss_clock_offset_ms').value))
        max_offset_ms = float(self.get_parameter('gnss_clock_max_offset_ms').value)
        self.s_ref_m += v_ref * self.period_s

        msg = FormationRef()
        msg.stamp = self.now_msg()
        msg.s_ref_m = self.s_ref_m
        msg.v_ref_mps = v_ref
        msg.role = FormationRef.ROLE_SINGLE_V1
        msg.gnss_clock_sync_ok = offset_ms <= max_offset_ms
        self.ref_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(FormationNode, args)
