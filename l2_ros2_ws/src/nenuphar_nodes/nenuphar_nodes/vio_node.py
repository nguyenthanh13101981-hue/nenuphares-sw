"""VIO estimator wrapper skeleton (VINS-Fusion-ROS2 selected at Milestone B)."""

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.qos import SENSOR_QOS


class VioNode(MonitoredNode):
    """Consumes stereo + IMU on SENSOR_QOS and publishes /vio/odometry."""

    def __init__(self) -> None:
        super().__init__('vio')
        self.left_sub = self.create_subscription(
            Image, '/camera/stereo/left', self._on_stereo_left, SENSOR_QOS
        )
        self.right_sub = self.create_subscription(
            Image, '/camera/stereo/right', self._on_stereo_right, SENSOR_QOS
        )
        self.imu_sub = self.create_subscription(
            Imu, '/camera/imu', self._on_imu, SENSOR_QOS
        )
        self.odom_pub = self.create_publisher(
            Odometry, '/vio/odometry', SENSOR_QOS
        )
        self.last_left_msg = None
        self.last_right_msg = None
        self.last_imu_msg = None

    def _on_stereo_left(self, msg: Image) -> None:
        self.last_left_msg = msg

    def _on_stereo_right(self, msg: Image) -> None:
        self.last_right_msg = msg

    def _on_imu(self, msg: Imu) -> None:
        self.last_imu_msg = msg
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self.odom_pub.publish(odom)


def main(args=None) -> None:
    run_node_main(VioNode, args)
