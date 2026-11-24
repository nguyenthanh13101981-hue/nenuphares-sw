"""Unit tests for MAVLink 2 signing, ENU->NED ODOMETRY transform, EKF source, and RX handling."""

import pytest
import rclpy
from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import MissionState

from nenuphar_nodes.fsm_engine import S5, S6
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


def test_mavlink_bridge_wires_odom_ekf_and_c8_signals() -> None:
    rclpy.init()
    node = MavlinkBridgeNode()
    try:
        assert node.tx_odom_count == 0
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        node._on_vio(odom)
        assert node.tx_odom_count == 1

        st5 = MissionState()
        st5.state = S5
        st5.bridge_permit = True
        node._on_fsm_state(st5)
        assert node.last_ekf_source_set == 2
        assert node.tx_c8_bridge_zone_count >= 1

        st6 = MissionState()
        st6.state = S6
        st6.bridge_permit = False
        node._on_fsm_state(st6)
        assert node.last_ekf_source_set == 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
