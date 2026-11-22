"""Geometric perception skeleton (B3-B6B, Config A 400P <= 50 ms on Pi 5)."""

from nenuphar_msgs.msg import ObstacleArray
from sensor_msgs.msg import Image

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import SENSOR_QOS


class PerceptionGeomNode(MonitoredNode):
    """Processes /camera/depth/image_rect and emits /perception/obstacles."""

    def __init__(self) -> None:
        super().__init__('perception_geom')
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect', self._on_depth, SENSOR_QOS
        )
        self.obstacles_pub = self.create_publisher(
            ObstacleArray, '/perception/obstacles', SENSOR_QOS
        )

    def _on_depth(self, msg: Image) -> None:
        out = ObstacleArray()
        out.stamp = msg.header.stamp
        out.frame_id = msg.header.frame_id or 'oakd_optical_frame'
        out.depth_config = ObstacleArray.CONFIG_A_400P
        self.obstacles_pub.publish(out)


def main(args=None) -> None:
    run_node_main(PerceptionGeomNode, args)
