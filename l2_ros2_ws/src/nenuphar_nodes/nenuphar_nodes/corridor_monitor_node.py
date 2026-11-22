"""Corridor Z1/Z2 deviation monitor (D-16: alert > 15 cm)."""

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import CorridorStatus, ObstacleArray

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_corridor_config
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


class CorridorMonitorNode(MonitoredNode):
    """Monitors lateral corridor error >= 10 Hz."""

    def __init__(self) -> None:
        super().__init__('corridor_monitor')
        self.cfg = load_corridor_config()
        self.alert_threshold_m = float(
            self.cfg['corridor']['lateral_deviation_alert_m']
        )
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
        self.timer = self.create_timer(0.1, self._publish_status)

    def _on_odom(self, msg: Odometry) -> None:
        self.lateral_error_m = float(msg.pose.pose.position.y)

    def _on_obstacles(self, _msg: ObstacleArray) -> None:
        pass

    def _publish_status(self) -> None:
        msg = CorridorStatus()
        msg.stamp = self.now_msg()
        msg.zone = CorridorStatus.ZONE_Z1
        msg.lateral_error_m = self.lateral_error_m
        msg.deviation_alert = abs(self.lateral_error_m) > self.alert_threshold_m
        self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(CorridorMonitorNode, args)
