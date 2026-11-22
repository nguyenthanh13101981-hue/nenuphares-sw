"""Integration tests for all 11 ROS 2 nodes and system_monitor -> mavlink_bridge chain."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64

from nenuphar_msgs.msg import ObstacleArray
from nenuphar_nodes.base_node import ALL_11_NODE_NAMES, MAVLINK_HEARTBEAT_TX_TOPIC
from nenuphar_nodes.blackbox_node import BlackboxNode
from nenuphar_nodes.camera_driver_node import CameraDriverNode
from nenuphar_nodes.clearance_node import ClearanceNode
from nenuphar_nodes.corridor_monitor_node import CorridorMonitorNode
from nenuphar_nodes.energy_node import EnergyNode
from nenuphar_nodes.formation_node import FormationNode
from nenuphar_nodes.mavlink_bridge_node import MavlinkBridgeNode
from nenuphar_nodes.mission_fsm_node import MissionFsmNode
from nenuphar_nodes.perception_geom_node import PerceptionGeomNode
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS
from nenuphar_nodes.system_monitor_node import SystemMonitorNode
from nenuphar_nodes.vio_node import VioNode


def test_11_nodes_qos_and_heartbeat_chain() -> None:
    rclpy.init()
    observer = None
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        cam = CameraDriverNode()
        vio = VioNode()
        geom = PerceptionGeomNode()
        clearance = ClearanceNode()
        fsm = MissionFsmNode()
        corridor = CorridorMonitorNode()
        formation = FormationNode()
        energy = EnergyNode()
        bridge = MavlinkBridgeNode()
        blackbox = BlackboxNode()
        sysmon = SystemMonitorNode()

        nodes = [
            cam,
            vio,
            geom,
            clearance,
            fsm,
            corridor,
            formation,
            energy,
            bridge,
            blackbox,
            sysmon,
        ]
        assert tuple(n.get_name() for n in nodes) == ALL_11_NODE_NAMES
        assert len(nodes) == 11

        # Verify depth=1 BEST_EFFORT QoS on sensor/perception publishers.
        sensor_pubs = [
            cam.depth_pub,
            cam.left_pub,
            cam.right_pub,
            cam.imu_pub,
            vio.odom_pub,
            geom.obstacles_pub,
        ]
        for pub in sensor_pubs:
            assert pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            assert pub.qos_profile.history == HistoryPolicy.KEEP_LAST
            assert pub.qos_profile.depth == 1

        received_tx = []
        observer = Node('hb_chain_observer')
        observer.create_subscription(
            UInt64,
            MAVLINK_HEARTBEAT_TX_TOPIC,
            lambda m: received_tx.append(m.data),
            CMD_QOS,
        )

        for n in nodes:
            executor.add_node(n)
        executor.add_node(observer)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if (
                received_tx
                and bridge.last_health is not None
                and bridge.last_health.all_nodes_alive
            ):
                break

        assert received_tx, 'Expected forwarded heartbeat on /mavlink/heartbeat_tx'
        assert bridge.tx_heartbeat_count >= 1
        assert bridge.last_health is not None
        assert bridge.last_health.usb3_camera_ok is True
        assert bridge.last_health.all_nodes_alive is True
        assert bridge.last_health.failed_node_mask == 0
    finally:
        for n in nodes:
            n.destroy_node()
        if observer is not None:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_mock_sensor_pipeline_flows_through_vio_and_perception() -> None:
    rclpy.init()
    cam = vio = geom = observer = None
    executor = SingleThreadedExecutor()
    try:
        cam = CameraDriverNode(
            parameter_overrides=[Parameter('enable_stub_stream', value=True)]
        )
        vio = VioNode()
        geom = PerceptionGeomNode()
        observer = Node('sensor_pipeline_observer')

        odom_msgs = []
        obstacle_msgs = []
        observer.create_subscription(
            Odometry, '/vio/odometry', lambda m: odom_msgs.append(m), SENSOR_QOS
        )
        observer.create_subscription(
            ObstacleArray,
            '/perception/obstacles',
            lambda m: obstacle_msgs.append(m),
            SENSOR_QOS,
        )

        for n in (cam, vio, geom, observer):
            executor.add_node(n)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if odom_msgs and obstacle_msgs:
                break

        assert odom_msgs, 'Expected Odometry on /vio/odometry from stub stream'
        assert obstacle_msgs, 'Expected ObstacleArray on /perception/obstacles'
        assert obstacle_msgs[-1].depth_config == ObstacleArray.CONFIG_A_400P
    finally:
        for n in (cam, vio, geom, observer):
            if n is not None:
                n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
