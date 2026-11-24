"""Corridor Z1/Z2 polyline monitor (TR-4.16 / D-16) with S11 avoidance awareness."""

from typing import Any, Dict, Tuple

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import CorridorStatus, MissionState, ObstacleArray
from std_msgs.msg import Bool

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_corridor_config
from nenuphar_nodes.corridor_logic import (
    ZONE_Z1,
    ZONE_Z2,
    CorridorTracker,
    project_onto_polyline,
)
from nenuphar_nodes.fsm_engine import S11
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS

__all__ = ['CorridorMonitorNode', 'CorridorTracker', 'evaluate_corridor', 'project_onto_polyline']


def evaluate_corridor(
    x_along_m: float,
    y_lateral_m: float,
    cfg: Dict[str, Any],
    z2_start_x_m: float = 100.0,
) -> Tuple[int, float, bool]:
    """Stateless helper evaluating Z1/Z2 zone and lateral deviation against TR-4.16 Z1 width."""
    del z2_start_x_m  # Z1 vs Z2 is defined by lateral corridor width per TR-4.16.
    z1_half = float(cfg['corridor'].get('z1_half_width_m', 0.30))
    zone = ZONE_Z2 if abs(y_lateral_m) > z1_half else ZONE_Z1
    deviation_alert = abs(y_lateral_m) > z1_half
    return zone, float(y_lateral_m), deviation_alert


class CorridorMonitorNode(MonitoredNode):
    """Monitors lateral corridor error >= 10 Hz across Z1 (+/-0.30m) and Z2 (+/-1.25m)."""

    def __init__(self, **kwargs) -> None:
        super().__init__('corridor_monitor', **kwargs)
        self.cfg = load_corridor_config()
        self.tracker = CorridorTracker(self.cfg)
        self.pos_x_m = 0.0
        self.pos_y_m = 0.0
        self.in_obstacle_avoid = False

        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_odom, SENSOR_QOS
        )
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/perception/obstacles', self._on_obstacles, SENSOR_QOS
        )
        self.fsm_sub = self.create_subscription(
            MissionState, '/fsm/state', self._on_fsm_state, CMD_QOS
        )
        self.status_pub = self.create_publisher(
            CorridorStatus, '/corridor/status', CMD_QOS
        )
        self.outside_z2_pub = self.create_publisher(
            Bool, '/corridor/outside_z2_hold', CMD_QOS
        )

        rate_hz = float(self.cfg['corridor'].get('monitor_rate_hz', 10.0))
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_status)

    def _on_odom(self, msg: Odometry) -> None:
        self.pos_x_m = float(msg.pose.pose.position.x)
        self.pos_y_m = float(msg.pose.pose.position.y)

    def _on_obstacles(self, _msg: ObstacleArray) -> None:
        pass

    def _on_fsm_state(self, msg: MissionState) -> None:
        self.in_obstacle_avoid = msg.state == S11

    def _publish_status(self) -> None:
        now_s = float(self.get_clock().now().nanoseconds) * 1e-9
        res = self.tracker.evaluate(
            self.pos_x_m,
            self.pos_y_m,
            now_s=now_s,
            in_obstacle_avoid=self.in_obstacle_avoid,
        )
        msg = CorridorStatus()
        msg.stamp = self.now_msg()
        msg.zone = CorridorStatus.ZONE_Z2 if res.zone == ZONE_Z2 else CorridorStatus.ZONE_Z1
        msg.lateral_error_m = res.lateral_error_m
        msg.deviation_alert = res.ddkc_alert
        self.status_pub.publish(msg)

        hold_msg = Bool()
        hold_msg.data = res.outside_z2_hold
        self.outside_z2_pub.publish(hold_msg)


def main(args=None) -> None:
    run_node_main(CorridorMonitorNode, args)
