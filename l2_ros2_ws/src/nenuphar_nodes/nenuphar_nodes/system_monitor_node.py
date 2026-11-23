"""System monitor aggregating node heartbeats and USB 3 camera check -> /system/health."""

from pathlib import Path
import time
from typing import Dict

from nenuphar_msgs.msg import SystemHealth
from std_msgs.msg import String

from nenuphar_nodes.base_node import (
    MONITORED_NODE_NAMES,
    NODE_HEARTBEAT_TOPIC,
    run_node_main,
)
from nenuphar_nodes.qos import CMD_QOS
from rclpy.node import Node


def _read_cpu_temp_c() -> float:
    thermal_path = Path('/sys/class/thermal/thermal_zone0/temp')
    try:
        raw = thermal_path.read_text(encoding='utf-8').strip()
        return float(raw) / 1000.0
    except OSError:
        return 0.0


class SystemMonitorNode(Node):
    """Aggregates node liveness into 1 Hz /system/health for mavlink_bridge & mission_fsm."""

    def __init__(self, **kwargs) -> None:
        super().__init__('system_monitor', **kwargs)
        self.declare_parameter('usb3_camera_ok', True)
        self.declare_parameter('heartbeat_timeout_s', 3.0)
        self.declare_parameter('publish_period_s', 1.0)

        self._last_seen: Dict[str, float] = {}
        self.hb_sub = self.create_subscription(
            String, NODE_HEARTBEAT_TOPIC, self._on_node_heartbeat, CMD_QOS
        )
        self.health_pub = self.create_publisher(
            SystemHealth, '/system/health', CMD_QOS
        )

        period = float(self.get_parameter('publish_period_s').value)
        self.timer = self.create_timer(period, self.publish_health)

    def _on_node_heartbeat(self, msg: String) -> None:
        name = msg.data.strip().lstrip('/')
        if name in MONITORED_NODE_NAMES:
            self._last_seen[name] = time.monotonic()

    def compute_failed_mask(self, now_monotonic: float) -> int:
        """Compute bitmask of nodes missing recent heartbeat."""
        timeout_s = float(self.get_parameter('heartbeat_timeout_s').value)
        mask = 0
        for idx, name in enumerate(MONITORED_NODE_NAMES):
            last = self._last_seen.get(name)
            if last is None or (now_monotonic - last) > timeout_s:
                mask |= 1 << idx
        return mask

    def publish_health(self) -> SystemHealth:
        """Build and publish SystemHealth message."""
        mask = self.compute_failed_mask(time.monotonic())
        msg = SystemHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.usb3_camera_ok = bool(self.get_parameter('usb3_camera_ok').value)
        msg.failed_node_mask = mask
        msg.all_nodes_alive = mask == 0
        msg.cpu_temp_c = _read_cpu_temp_c()
        self.health_pub.publish(msg)
        return msg


def main(args=None) -> None:
    run_node_main(SystemMonitorNode, args)
