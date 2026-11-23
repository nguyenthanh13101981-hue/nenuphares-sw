"""Unit tests for MAVLink 2 signing, ODOMETRY forwarding, and EKF source command."""

import pytest
import rclpy
from nav_msgs.msg import Odometry

from nenuphar_nodes.mavlink_bridge_node import (
    MavlinkBridgeNode,
    configure_mavlink2_signing,
    parse_signing_key,
)


class _DummyMavConn:
    def __init__(self) -> None:
        self.called = None

    def setup_signing(self, key: bytes, sign_outgoing: bool = True, link_id: int = 0):
        self.called = (key, sign_outgoing, link_id)


def test_mavlink2_signing_key_parsing_and_setup() -> None:
    key_hex = '01' * 32
    key_bytes = parse_signing_key(key_hex)
    assert len(key_bytes) == 32

    dummy = _DummyMavConn()
    configure_mavlink2_signing(dummy, key_bytes, link_id=3)
    assert dummy.called == (key_bytes, True, 3)

    with pytest.raises(ValueError):
        parse_signing_key('deadbeef')


def test_mavlink_bridge_counts_vio_odom_and_ekf_source_set() -> None:
    rclpy.init()
    node = MavlinkBridgeNode()
    try:
        assert node.tx_odom_count == 0
        node._on_vio(Odometry())
        assert node.tx_odom_count == 1
        node.send_ekf_source_set(2)
        assert node.last_ekf_source_set == 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
