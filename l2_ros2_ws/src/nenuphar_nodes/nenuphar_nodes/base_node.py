"""Shared base utilities and liveness heartbeat for NENUPHARES L2 nodes."""

from typing import Tuple

from builtin_interfaces.msg import Time
from rclpy.node import Node
from std_msgs.msg import String

from nenuphar_nodes.qos import CMD_QOS

NODE_HEARTBEAT_TOPIC = '/system/node_heartbeat'
MAVLINK_HEARTBEAT_TX_TOPIC = '/mavlink/heartbeat_tx'

# 10 nodes monitored by system_monitor (bit index = position in tuple).
MONITORED_NODE_NAMES: Tuple[str, ...] = (
    'camera_driver',
    'vio',
    'perception_geom',
    'clearance',
    'mission_fsm',
    'corridor_monitor',
    'formation',
    'energy',
    'mavlink_bridge',
    'blackbox',
)

ALL_11_NODE_NAMES: Tuple[str, ...] = MONITORED_NODE_NAMES + ('system_monitor',)


def run_node_main(node_cls, args=None) -> None:
    """Standard lifecycle entrypoint for single-node executables."""
    import rclpy

    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class MonitoredNode(Node):
    """Base Node emitting 1 Hz liveness name to /system/node_heartbeat."""

    def __init__(self, node_name: str, emit_heartbeat: bool = True) -> None:
        super().__init__(node_name)
        self._liveness_pub = None
        self._liveness_timer = None
        if emit_heartbeat:
            self._liveness_pub = self.create_publisher(
                String, NODE_HEARTBEAT_TOPIC, CMD_QOS
            )
            self._liveness_timer = self.create_timer(1.0, self._emit_liveness)

    def _emit_liveness(self) -> None:
        if self._liveness_pub is None:
            return
        msg = String()
        msg.data = self.get_name()
        self._liveness_pub.publish(msg)

    def now_msg(self) -> Time:
        """Return current ROS clock timestamp as builtin_interfaces/Time."""
        return self.get_clock().now().to_msg()
