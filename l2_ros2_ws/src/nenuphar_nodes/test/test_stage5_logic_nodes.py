"""Stage 5 unit tests: S0-S12 FSM transitions, 3-tier faults, corridor, energy, formation."""

import rclpy
from rclpy.parameter import Parameter

from nenuphar_msgs.msg import CorridorStatus, EnergyStatus, FormationRef
from nenuphar_nodes.config_loader import load_corridor_config, load_energy_config
from nenuphar_nodes.corridor_monitor_node import evaluate_corridor
from nenuphar_nodes.energy_node import EnergyNode, integrate_shunt_wh
from nenuphar_nodes.formation_node import FormationNode
from nenuphar_nodes.fsm_engine import (
    ALLOWED_TRANSITIONS,
    FsmConfig,
    FsmInputs,
    MissionFsmEngine,
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12,
)


def test_corridor_z1_z2_and_15cm_alert() -> None:
    cfg = load_corridor_config()
    zone, err, alert = evaluate_corridor(25.0, 0.14, cfg, z2_start_x_m=100.0)
    assert zone == CorridorStatus.ZONE_Z1
    assert abs(err - 0.14) < 1e-6
    assert alert is False

    zone, err, alert = evaluate_corridor(120.0, -0.16, cfg, z2_start_x_m=100.0)
    assert zone == CorridorStatus.ZONE_Z2
    assert abs(err + 0.16) < 1e-6
    assert alert is True


def test_energy_shunt_integration_and_ble_loss_warn_only_tbd23() -> None:
    cfg = load_energy_config()
    total_wh = float(cfg['energy_management']['total_capacity_wh'])
    assert total_wh == 1280.0

    # 12.8 V * 10 A for 3600 s = 128 Wh consumed -> 1152 Wh remaining.
    rem = integrate_shunt_wh(total_wh, 12.8, 10.0, 3600.0, total_wh)
    assert abs(rem - 1152.0) < 1e-3

    # TBD-23: BMS Bluetooth LE loss must only warn and NEVER trigger STOP.
    rclpy.init()
    node = EnergyNode(
        parameter_overrides=[
            Parameter('bms_ble_connected', value=False),
            Parameter('stub_soc_pct', value=65.0),
            Parameter('stub_voltage_v', value=13.1),
        ]
    )
    try:
        received = []
        node.status_pub.publish = lambda m: received.append(m)  # type: ignore[method-assign]
        node._publish_status()
        assert node.ble_warn_count == 1
        assert len(received) == 1
        assert received[0].bms_ble_connected is False
        assert received[0].level == EnergyStatus.LEVEL_OK
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_formation_v1_minimal_rate_and_gnss_clock_sync() -> None:
    rclpy.init()
    node = FormationNode(
        parameter_overrides=[
            Parameter('rate_hz', value=2.0),
            Parameter('nominal_speed_mps', value=1.0),
            Parameter('gnss_clock_offset_ms', value=4.0),
        ]
    )
    try:
        msgs = []
        node.ref_pub.publish = lambda m: msgs.append(m)  # type: ignore[method-assign]
        node._publish_ref()
        node._publish_ref()
        assert len(msgs) == 2
        assert msgs[-1].role == FormationRef.ROLE_SINGLE_V1
        assert msgs[-1].s_ref_m == 1.0
        assert msgs[-1].gnss_clock_sync_ok is True

        node.set_parameters([Parameter('gnss_clock_offset_ms', value=15.0)])
        node._publish_ref()
        assert msgs[-1].gnss_clock_sync_ok is False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_fsm_three_fault_tiers_and_usb3_gate() -> None:
    eng = MissionFsmEngine(FsmConfig(s3_timeout_s=30.0, s6_timeout_s=20.0))

    # USB 3 camera missing -> S0 rejects starting mission.
    out = eng.step(FsmInputs(now_s=1.0, usb3_camera_ok=False, start_mission=True))
    assert out.state == S0

    # Precheck passes -> S0 -> S1.
    out = eng.step(FsmInputs(now_s=2.0, usb3_camera_ok=True, start_mission=True))
    assert out.state == S1 and out.fault_tier == 0

    # Tier 1 fault (BMS BLE disconnected + WARN): stays in S1, fault_tier=1, self-recovers.
    out = eng.step(
        FsmInputs(now_s=3.0, bms_ble_connected=False, energy_level=EnergyStatus.LEVEL_WARN)
    )
    assert out.state == S1 and out.fault_tier == 1 and out.hold_all_req is False

    out = eng.step(FsmInputs(now_s=4.0, bms_ble_connected=True, energy_level=EnergyStatus.LEVEL_OK))
    assert out.state == S1 and out.fault_tier == 0

    # Tier 2 fault (corridor deviation > 15 cm) -> S12 DDKC.
    out = eng.step(FsmInputs(now_s=5.0, corridor_deviation_alert=True))
    assert out.state == S12 and out.fault_tier == 2 and out.hold_position is True

    # Tier 3 critical fault -> S8 Safe Hold + HOLD-ALL <= 2 s.
    out = eng.step(FsmInputs(now_s=6.0, tier3_critical_fault=True))
    assert out.state == S8 and out.fault_tier == 3
    assert out.hold_all_req is True and out.hold_all_latency_s <= 2.0

    # Highest-priority S9 (battery STOP <= 10%) preempts S8.
    out = eng.step(FsmInputs(now_s=7.0, energy_level=EnergyStatus.LEVEL_STOP))
    assert out.state == S9 and out.fault_tier == 3 and out.hold_all_req is True
    assert len(eng.invalid_transitions) == 0


def test_fsm_s3_s6_timeouts_and_bridge_speed_limit() -> None:
    eng = MissionFsmEngine(FsmConfig(s3_timeout_s=30.0, s6_timeout_s=20.0))
    eng.step(FsmInputs(now_s=0.0, start_mission=True))
    out_s3 = eng.step(FsmInputs(now_s=1.0, bridge_detected=True))
    assert out_s3.state == S3
    assert out_s3.max_speed_mps <= (2.0 / 3.6) + 1e-6

    # S3 timeout -> S12.
    out_to = eng.step(FsmInputs(now_s=32.0))
    assert out_to.state == S12 and out_to.fault_tier == 2
    assert (S3, S12) in eng.observed_transitions

    # Reset and verify S6 timeout -> S12.
    eng.step(FsmInputs(now_s=33.0, reset_to_s0=True))
    eng.step(FsmInputs(now_s=34.0, start_mission=True))
    eng.step(FsmInputs(now_s=35.0, bridge_detected=True))
    eng.step(FsmInputs(now_s=36.0, at_bridge_measure_point=True))
    out_s5 = eng.step(
        FsmInputs(now_s=37.0, bridge_clearance_ready=True, bridge_pass_permitted=True)
    )
    assert out_s5.state == S5 and out_s5.ekf_source_set == 2 and out_s5.bridge_zone_active
    eng.step(FsmInputs(now_s=38.0, bridge_exited=True))
    assert eng.state == S6
    out_s6_to = eng.step(FsmInputs(now_s=60.0))
    assert out_s6_to.state == S12
    assert (S6, S12) in eng.observed_transitions
    assert len(eng.invalid_transitions) == 0
    assert len(ALLOWED_TRANSITIONS) == 41
