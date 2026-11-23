"""Corridor Z1/Z2 deviation monitor (D-16: alert > 15 cm)."""

from typing import Any, Dict, Tuple

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import CorridorStatus, ObstacleArray

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_corridor_config
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


def evaluate_corridor(
    x_along_m: float,
    y_lateral_m: float,
    cfg: Dict[str, Any],
    z2_start_x_m: float = 100.0,
) -> Tuple[int, float, bool]:
    """Compute active corridor zone (Z1/Z2), lateral error, and >15 cm deviation alert."""
    threshold = float(cfg['corridor']['lateral_deviation_alert_m'])
    zone = CorridorStatus.ZONE_Z2 if x_along_m >= z2_start_x_m else CorridorStatus.ZONE_Z1
    deviation_alert = abs(y_lateral_m) > threshold
    return zone, float(y_lateral_m), deviation_alert


class CorridorMonitorNode(MonitoredNode):
    """Monitors lateral corridor error >= 10 Hz across Z1/Z2."""

    def __init__(self, **kwargs) -> None:
        super().__init__('corridor_monitor', **kwargs)
        self.cfg = load_corridor_config()
        self.declare_parameter('z2_start_x_m', 100.0)
        self.z2_start_x_m = float(self.get_parameter('z2_start_x_m').value)
        self.along_x_m = 0.0
        self.lateral_error_m = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_odom, SENSOR_QOS
        )
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/perception/obstacles', self._on_obstacles, SENSOR_QOS
        )
        self.status_pub = self.create_publisher(
            CorridorStatus, '/corridor/status', CMD_QOS
        )
        rate_hz = float(self.cfg['corridor'].get('monitor_rate_hz', 10.0))
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_status)

    def _on_odom(self, msg: Odometry) -> None:
        self.along_x_m = float(msg.pose.pose.position.x)
        self.lateral_error_m = float(msg.pose.pose.position.y)

    def _on_obstacles(self, _msg: ObstacleArray) -> None:
        pass

    def _publish_status(self) -> None:
        zone, lat_err, alert = evaluate_corridor(
            self.along_x_m, self.lateral_error_m, self.cfg, self.z2_start_x_m
        )
        msg = CorridorStatus()
        msg.stamp = self.now_msg()
        msg.zone = zone
        msg.lateral_error_m = lat_err
        msg.deviation_alert = alert
        self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(CorridorMonitorNode, args)
