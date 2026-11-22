"""Unit tests for QoS profiles and Final 2.0 configuration files."""

from rclpy.qos import HistoryPolicy, ReliabilityPolicy

from nenuphar_msgs.msg import EnergyStatus
from nenuphar_nodes.config_loader import (
    load_corridor_config,
    load_energy_config,
    load_raft_profile,
)
from nenuphar_nodes.energy_node import classify_energy_level
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


def test_qos_profiles_match_interface_freeze() -> None:
    assert SENSOR_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert SENSOR_QOS.history == HistoryPolicy.KEEP_LAST
    assert SENSOR_QOS.depth == 1

    assert CMD_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert CMD_QOS.history == HistoryPolicy.KEEP_LAST
    assert CMD_QOS.depth == 10


def test_raft_profiles_and_final20_invariants() -> None:
    p70 = load_raft_profile('v1_70x50')
    p98 = load_raft_profile('v1_98x68')

    assert p70['hull']['length_m'] == 0.70
    assert p70['hull']['width_m'] == 0.50
    assert p98['hull']['length_m'] == 0.98
    assert p98['hull']['width_m'] == 0.68

    for prof in (p70, p98):
        cam = prof['camera_oak_d_pro_w']
        assert cam['mount_height_above_deck_m'] == 0.20
        assert cam['alpha_tilt_deg'] == 0.0
        assert cam['depth_hz'] == 15.0
        assert cam['imu_hz'] == 200.0
        assert prof['gnss_dual_antenna']['use_compass'] is False
        assert prof['thrusters']['esc_count'] == 3
        assert set(prof['thrusters']['units'].keys()) == {
            'left',
            'right',
            'stern_backup',
        }


def test_energy_and_corridor_configs() -> None:
    energy_cfg = load_energy_config()
    em = energy_cfg['energy_management']
    assert em['total_capacity_wh'] == 1280.0
    assert em['primary_measurement'] == 'shunt'
    assert em['bms_interface'] == 'bluetooth_le'
    assert em['bms_ble_loss_policy'] == 'warn_only_no_stop'

    assert classify_energy_level(50.0, 13.0, energy_cfg) == EnergyStatus.LEVEL_OK
    assert classify_energy_level(28.0, 12.6, energy_cfg) == EnergyStatus.LEVEL_WARN
    assert classify_energy_level(21.0, 12.2, energy_cfg) == EnergyStatus.LEVEL_RTB
    assert classify_energy_level(10.0, 12.1, energy_cfg) == EnergyStatus.LEVEL_STOP
    assert classify_energy_level(40.0, 11.5, energy_cfg) == EnergyStatus.LEVEL_STOP

    corridor_cfg = load_corridor_config()
    assert corridor_cfg['corridor']['lateral_deviation_alert_m'] == 0.15
    assert corridor_cfg['bridge_s4']['max_speed_kmh'] == 2.0
    assert corridor_cfg['obstacle_avoidance']['e2e_latency_budget_ms'] == 200.0
