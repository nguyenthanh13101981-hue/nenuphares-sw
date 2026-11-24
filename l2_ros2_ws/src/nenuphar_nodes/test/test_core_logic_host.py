"""Pure-Python host unit & property tests for CdCT A.8 FSM, transforms, corridor, energy, clearance, USB 3."""

import math
from pathlib import Path
import tempfile

from nenuphar_nodes.clearance_logic import evaluate_bridge_clearance
from nenuphar_nodes.config_loader import (
    load_cdct_spec,
    load_corridor_config,
    load_energy_config,
    load_raft_profile,
)
from nenuphar_nodes.corridor_logic import (
    ZONE_Z1,
    ZONE_Z2,
    CorridorTracker,
    project_onto_polyline,
)
from nenuphar_nodes.energy_logic import (
    LEVEL_OK,
    LEVEL_RTB,
    LEVEL_STOP,
    LEVEL_WARN,
    EnergyClassifier,
    integrate_shunt_wh,
)
from nenuphar_nodes.fsm_engine import (
    ALL_STATES,
    ALLOWED_TRANSITIONS,
    PRIORITY_ORDER,
    STATE_NAMES,
    FsmConfig,
    FsmInputs,
    MissionFsmEngine,
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12,
    resolve_fsm_state,
)
from nenuphar_nodes.mavlink_transforms import (
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_LOCAL_NED,
    ros_enu_odom_to_mavlink_ned,
)
from nenuphar_nodes.usb_probe import detect_oakd_usb3_sysfs


def test_cdct_spec_a8_definitions_and_priority() -> None:
    spec = load_cdct_spec()
    assert len(ALL_STATES) == 13
    assert PRIORITY_ORDER == (S9, S8, S12, S7, S11, S10, S1)
    assert STATE_NAMES[S2] == 'BRIDGE_APPROACH'
    assert STATE_NAMES[S3] == 'BRIDGE_CENTERING'
    assert STATE_NAMES[S7] == 'HOLD_WAIT'
    assert STATE_NAMES[S8] == 'BRIDGE_EMERGENCY_EXIT'
    assert STATE_NAMES[S11] == 'OBSTACLE_AVOID'
    assert len(ALLOWED_TRANSITIONS) == len(spec['a8_fsm']['allowed_transitions'])


def _advance_to_state(target_state: int) -> MissionFsmEngine:
    eng = MissionFsmEngine()
    if target_state == S0:
        return eng
    eng.step(FsmInputs(now_s=1.0, start_mission=True))
    if target_state == S1:
        return eng
    if target_state == S11:
        eng.step(FsmInputs(now_s=2.0, obstacle_detected=True, obstacle_gap_m=1.5))
        return eng
    if target_state == S10:
        eng.step(FsmInputs(now_s=2.0, rtb_requested=True))
        return eng
    if target_state == S7:
        eng.step(FsmInputs(now_s=2.0, hold_requested=True))
        return eng
    if target_state == S12:
        eng.step(FsmInputs(now_s=2.0, corridor_ddkc_alert=True))
        return eng
    # Bridge states S2..S6 and S8
    eng.step(FsmInputs(now_s=2.0, bridge_detected=True))
    if target_state == S2:
        return eng
    eng.step(FsmInputs(now_s=3.0, at_bridge_centering=True))
    if target_state == S3:
        return eng
    eng.step(FsmInputs(now_s=4.0, bridge_centered=True))
    if target_state == S4:
        return eng
    eng.step(
        FsmInputs(
            now_s=5.0,
            bridge_clearance_ready=True,
            bridge_pass_permitted=True,
            bridge_id='b1',
        )
    )
    if target_state == S5:
        return eng
    if target_state == S8:
        eng.step(FsmInputs(now_s=6.0, tier3_critical_fault=True))
        return eng
    if target_state == S6:
        eng.step(FsmInputs(now_s=6.0, bridge_exited=True))
        return eng
    if target_state == S9:
        eng.step(FsmInputs(now_s=6.0, emergency_stop_cmd=True))
        return eng
    raise AssertionError(f'Unhandled state {target_state}')


def test_property_tier3_fault_never_ignored_from_any_state() -> None:
    """Property (A9): From EVERY state S0..S12, Tier 3 fault MUST land in {S8, S9} with 0 invalid edges."""
    for src in ALL_STATES:
        eng = _advance_to_state(src)
        assert eng.state == src
        out = eng.step(
            FsmInputs(
                now_s=10.0,
                fault_detected_s=9.95,
                tier3_critical_fault=True,
                all_nodes_alive=False if src != S0 else True,
            )
        )
        assert out.state in (S8, S9), f'Tier 3 ignored in state S{src} -> S{out.state}'
        assert out.hold_all_req is True
        assert abs(out.hold_all_latency_s - 0.05) < 1e-6
        assert eng.invalid_transitions == []

        eng2 = _advance_to_state(src)
        out2 = eng2.step(FsmInputs(now_s=10.0, emergency_stop_cmd=True))
        assert out2.state == S9
        assert eng2.invalid_transitions == []


def test_cdct_a8_bridge_obstacle_ddkc_and_stale_clearance_cleared() -> None:
    eng = MissionFsmEngine(FsmConfig(s3_centering_timeout_s=30.0, s6_rtk_timeout_s=30.0))

    # USB 3 gate blocks leaving S0.
    assert eng.step(FsmInputs(now_s=1.0, usb3_camera_ok=False, start_mission=True)).state == S0
    assert eng.step(FsmInputs(now_s=2.0, usb3_camera_ok=True, start_mission=True)).state == S1

    # S11 OBSTACLE_AVOID preempts S10 RTB (S11 > S10).
    out = eng.step(
        FsmInputs(now_s=3.0, rtb_requested=True, obstacle_detected=True, obstacle_gap_m=1.40)
    )
    assert out.state == S11
    assert eng.step(FsmInputs(now_s=4.0, rtb_requested=True, obstacle_detected=False)).state == S10
    assert eng.step(FsmInputs(now_s=5.0, docked_at_base=True)).state == S7
    assert eng.step(FsmInputs(now_s=6.0, reset_to_s0=True)).state == S0

    # Bridge sequence: S1 -> S2 -> S3 -> S4 denied -> S7; retry S7 -> S4 -> S5 -> S8 -> S7.
    eng.step(FsmInputs(now_s=7.0, start_mission=True))
    assert eng.step(FsmInputs(now_s=8.0, bridge_detected=True)).state == S2
    assert eng.step(FsmInputs(now_s=9.0, at_bridge_centering=True)).state == S3
    assert eng.step(FsmInputs(now_s=10.0, bridge_centered=True)).state == S4
    # Unpermitted S4 -> S7 (NOT S12 immediately per CdCT A.8!).
    out_wait = eng.step(
        FsmInputs(now_s=11.0, bridge_clearance_ready=True, bridge_pass_permitted=False)
    )
    assert out_wait.state == S7
    assert out_wait.bridge_permit is False

    # Re-enter S4 -> granted -> S5 -> Tier 3 under bridge with propulsion -> S8!
    assert (
        eng.step(
            FsmInputs(now_s=12.0, bridge_detected=True, at_bridge_centering=True, bridge_centered=True)
        ).state
        == S4
    )
    out_s5 = eng.step(
        FsmInputs(now_s=13.0, bridge_clearance_ready=True, bridge_pass_permitted=True, bridge_id='br2')
    )
    assert out_s5.state == S5 and out_s5.ekf_source_set == 2 and out_s5.bridge_permit is True
    out_s8 = eng.step(FsmInputs(now_s=14.0, tier3_critical_fault=True, propulsion_available=True))
    assert out_s8.state == S8 and out_s8.ekf_source_set == 2 and out_s8.bridge_permit is False

    # B6: SoC <= 10% with propulsion goes to S12 DDKC (not immediate motor kill S9).
    eng2 = _advance_to_state(S1)
    out_soc = eng2.step(FsmInputs(now_s=5.0, energy_level=LEVEL_STOP, propulsion_available=True))
    assert out_soc.state == S12 and out_soc.fault_tier == 2

    # Check resolve_fsm_state priority order.
    assert resolve_fsm_state(S1, True, (S10, S11)) == S11
    assert resolve_fsm_state(S0, False, (S1, S11)) == S0


def test_mavlink_enu_ned_transform_preserves_w0_and_rotations() -> None:
    # Pose 1: East-facing in ENU (yaw=0 -> q=[0,0,0,1]), moving forward +1.5 m/s, left +0.2 m/s.
    p_east = ros_enu_odom_to_mavlink_ned(
        pos_enu=(10.0, 25.0, 1.5),
        quat_xyzw_enu_flu=(0.0, 0.0, 0.0, 1.0),
        vel_flu=(1.5, 0.2, 0.1),
        ang_vel_flu=(0.0, 0.0, 0.3),
        stamp_sec=123,
        stamp_nanosec=456_000_000,
    )
    assert p_east.frame_id == MAV_FRAME_LOCAL_NED
    assert p_east.child_frame_id == MAV_FRAME_BODY_FRD
    assert p_east.time_usec == 123_456_000
    assert (p_east.x_ned, p_east.y_ned, p_east.z_ned) == (25.0, 10.0, -1.5)
    assert (p_east.vx_frd, p_east.vy_frd, p_east.vz_frd) == (1.5, -0.2, -0.1)
    assert (p_east.rollspeed_frd, p_east.pitchspeed_frd, p_east.yawspeed_frd) == (0.0, 0.0, -0.3)
    # Facing East in NED is +90 deg yaw -> q_wxyz = [1/sqrt(2), 0, 0, 1/sqrt(2)]
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    assert abs(p_east.q_wxyz[0] - inv_sqrt2) < 1e-6
    assert abs(p_east.q_wxyz[3] - inv_sqrt2) < 1e-6
    assert all(math.isfinite(c) for c in p_east.pose_covariance)

    # Pose 2: West-facing in ENU (180 deg yaw -> q_xyzw = [0, 0, 1, 0], where w == 0.0!).
    p_west = ros_enu_odom_to_mavlink_ned(
        pos_enu=(0.0, 0.0, 0.0),
        quat_xyzw_enu_flu=(0.0, 0.0, 1.0, 0.0),
        vel_flu=(0.0, 0.0, 0.0),
        ang_vel_flu=(0.0, 0.0, 0.0),
        stamp_sec=1,
        stamp_nanosec=0,
    )
    # Facing West in NED is -90 deg yaw -> q_wxyz = [1/sqrt(2), 0, 0, -1/sqrt(2)]
    assert abs(p_west.q_wxyz[0] - inv_sqrt2) < 1e-6
    assert abs(p_west.q_wxyz[3] + inv_sqrt2) < 1e-6


def test_corridor_polyline_tr416_timer_retry_and_s11_avoidance() -> None:
    cfg = load_corridor_config()
    # Diagonal polyline from (0,0) to (100,100).
    s, lat, hdg = project_onto_polyline(50.0, 50.0, [[0.0, 0.0], [100.0, 100.0]])
    assert abs(lat) < 1e-6 and s > 70.0 and abs(hdg - math.pi / 4.0) < 1e-6

    tracker = CorridorTracker(cfg, waypoints=[[0.0, 0.0], [200.0, 0.0]])
    # Inside Z1 (<= 0.30 m)
    r0 = tracker.evaluate(10.0, 0.25, now_s=0.0)
    assert r0.zone == ZONE_Z1 and not r0.ddkc_alert and not r0.outside_z2_hold

    # Excursion to 0.42 m (Z2) for <= 5 s -> self-correction, no DDKC yet.
    r1 = tracker.evaluate(15.0, 0.42, now_s=1.0)
    assert r1.zone == ZONE_Z2 and r1.self_correction_active and not r1.ddkc_alert
    r2 = tracker.evaluate(18.0, 0.42, now_s=5.5)
    assert not r2.ddkc_alert

    # > 5 s first window triggers 1 retry; > 5 s again triggers DDKC!
    r3 = tracker.evaluate(20.0, 0.42, now_s=6.5)
    assert r3.self_correction_active and not r3.ddkc_alert
    r4 = tracker.evaluate(25.0, 0.42, now_s=12.0)
    assert r4.ddkc_alert is True

    # During S11 obstacle avoidance (y=0.90 m inside Z2 <= 1.25 m): NO false DDKC!
    tracker.reset()
    r_avoid = tracker.evaluate(30.0, 0.90, now_s=20.0, in_obstacle_avoid=True)
    assert not r_avoid.ddkc_alert and not r_avoid.outside_z2_hold

    # Beyond Z2 (> 1.25 m) -> immediate Hold (S7).
    r_out = tracker.evaluate(35.0, 1.30, now_s=21.0, in_obstacle_avoid=True)
    assert r_out.outside_z2_hold is True


def test_energy_a93_hysteresis_clearance_b2_and_usb3_sysfs() -> None:
    ecfg = load_energy_config()
    clf = EnergyClassifier(ecfg)
    assert clf.classify(31.0, 13.0) == LEVEL_OK
    assert clf.classify(28.0, 12.6) == LEVEL_WARN
    # Hysteresis: 29% stays WARN until >= 30%
    assert clf.classify(29.0, 12.7) == LEVEL_WARN
    assert clf.classify(30.0, 12.8) == LEVEL_OK
    assert clf.classify(21.0, 12.2) == LEVEL_RTB
    assert clf.classify(10.0, 12.1) == LEVEL_STOP
    assert abs(integrate_shunt_wh(1280.0, 12.8, 10.0, 3600.0, 1280.0) - 1152.0) < 1e-3

    prof = load_raft_profile('v1_70x50')
    raft_h = float(prof['hull']['total_height_above_water_m'])
    assert evaluate_bridge_clearance(raft_h + 0.15, 0.03, raft_h, db_clearance_m=raft_h + 0.14) is True
    # Too low (< raft_h + 0.10 m)
    assert evaluate_bridge_clearance(raft_h + 0.05, 0.02, raft_h) is False
    # DB mismatch > 0.05 m
    assert evaluate_bridge_clearance(raft_h + 0.50, 0.02, raft_h, db_clearance_m=raft_h + 0.40) is False

    # Mock sysfs USB 2 (480 Mbps) vs USB 3 (5000 Mbps)
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        dev1 = root / '1-1'
        dev1.mkdir()
        (dev1 / 'idVendor').write_text('03e7\n', encoding='utf-8')
        (dev1 / 'speed').write_text('480\n', encoding='utf-8')
        ok2, spd2 = detect_oakd_usb3_sysfs(root)
        assert ok2 is False and spd2 == 480.0

        dev2 = root / '2-1'
        dev2.mkdir()
        (dev2 / 'idVendor').write_text('03e7\n', encoding='utf-8')
        (dev2 / 'speed').write_text('5000\n', encoding='utf-8')
        ok3, spd3 = detect_oakd_usb3_sysfs(root)
        assert ok3 is True and spd3 == 5000.0
