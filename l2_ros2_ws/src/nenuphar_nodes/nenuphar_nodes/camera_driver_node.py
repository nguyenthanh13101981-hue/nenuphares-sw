"""OAK-D Pro W camera driver skeleton (15 Hz depth/stereo, 200 Hz IMU, horizontal mount)."""

from sensor_msgs.msg import Image, Imu

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_raft_profile
from nenuphar_nodes.qos import SENSOR_QOS


class CameraDriverNode(MonitoredNode):
    """Publishes OAK-D Pro W streams using non-blocking SENSOR_QOS (depth=1)."""

    def __init__(self, **kwargs) -> None:
        super().__init__('camera_driver', **kwargs)
        self.declare_parameter('raft_profile', 'v1_70x50')
        self.declare_parameter('enable_stub_stream', False)

        profile_name = str(self.get_parameter('raft_profile').value)
        self.raft_profile = load_raft_profile(profile_name)
        cam_cfg = self.raft_profile['camera_oak_d_pro_w']
        if abs(float(cam_cfg['alpha_tilt_deg'])) > 1e-6:
            raise ValueError('Final 2.0 rule violation: alpha_tilt_deg must be 0.0')

        self.depth_pub = self.create_publisher(
            Image, '/camera/depth/image_rect', SENSOR_QOS
        )
        self.left_pub = self.create_publisher(
            Image, '/camera/stereo/left', SENSOR_QOS
        )
        self.right_pub = self.create_publisher(
            Image, '/camera/stereo/right', SENSOR_QOS
        )
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', SENSOR_QOS)

        self._stream_timer = None
        if bool(self.get_parameter('enable_stub_stream').value):
            depth_hz = float(cam_cfg.get('depth_hz', 15.0))
            self._stream_timer = self.create_timer(
                1.0 / depth_hz, self._publish_stub_frames
            )

    def _publish_stub_frames(self) -> None:
        stamp = self.now_msg()
        for pub in (self.depth_pub, self.left_pub, self.right_pub):
            img = Image()
            img.header.stamp = stamp
            img.header.frame_id = 'oakd_optical_frame'
            pub.publish(img)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'oakd_imu_frame'
        self.imu_pub.publish(imu)


def main(args=None) -> None:
    run_node_main(CameraDriverNode, args)
