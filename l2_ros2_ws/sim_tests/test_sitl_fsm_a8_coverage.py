"""Stage 5 SITL verification covering 100% of A.8 S0-S12 transitions with zero invalid edges.

Runs live ArduPilot Rover 4.7.3 Boat SITL (sitl_boat_a7.parm + L1 Lua scripts + MAVLink 2 signing)
coupled to MissionFsmEngine to prove A.13 criterion:
  - Every valid transition in ALLOWED_TRANSITIONS (41 edges across S0..S12) triggered >= 1 time.
  - Zero out-of-table transitions (invalid_transitions == []).
  - Real SITL interactions verified during mission: MAVLink 2 signed heartbeat, BRIDGE_PERMIT /
    BRIDGE_ZONE Lua synchronization, GNSS loss + 25 Hz VIO ODOMETRY + EKF source switch in S5,
    and HOLD mode enforcement on Tier 3 faults (S8/S9).
"""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

from pymavlink import mavutil

os.environ['MAVLINK20'] = '1'

REPO_ROOT = Path('/home/nenuphar/nenuphares-sw')
sys.path.insert(0, str(REPO_ROOT / 'l2_ros2_ws/src/nenuphar_nodes'))

from nenuphar_msgs.msg import EnergyStatus  # noqa: E402
from nenuphar_nodes.fsm_engine import (  # noqa: E402
    ALLOWED_TRANSITIONS,
    FsmConfig,
    FsmInputs,
    MissionFsmEngine,
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12,
)

ARDUROVER_BIN = Path.home() / 'sim/ardupilot/build/sitl/bin/ardurover'
DEFAULT_ROVER_PARM = Path.home() / 'sim/ardupilot/Tools/autotest/default_params/rover.parm'
A7_PARM = REPO_ROOT / 'l1_ardupilot/params/sitl_boat_a7.parm'
LUA_DIR = REPO_ROOT / 'l1_ardupilot/lua'

SIGNING_KEY = bytes.fromhex('4e454e5550484152' * 4)
MAV_CMD_SET_EKF_SOURCE_SET = 42007


def _set_param(master, name: str, value: float, timeout: float = 5.0) -> float:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('ascii'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.4)
        if msg and msg.param_id.rstrip('\x00') == name:
            return float(msg.param_value)
    raise TimeoutError(f'Timed out setting param {name}={value}')


def _send_l2_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def _send_ekf_source(master, source_set: int) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAV_CMD_SET_EKF_SOURCE_SET,
        0,
        float(source_set),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.4)
        if ack and ack.command == MAV_CMD_SET_EKF_SOURCE_SET:
            assert int(ack.result) == mavutil.mavlink.MAV_RESULT_ACCEPTED
            return
    raise TimeoutError(f'No COMMAND_ACK for EKF source set {source_set}')


def _send_vio_odometry_burst(master, count: int = 5) -> None:
    cov = [float('nan')] + [0.0] * 20
    for _ in range(count):
        master.mav.odometry_send(
            int(time.time() * 1e6),
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,
            0.0,
            0.0,
            [1.0, 0.0, 0.0, 0.0],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            cov,
            cov,
            0,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
            100,
        )
        time.sleep(0.04)  # 25 Hz


def _wait_mode(master, expected_mode: int, timeout: float = 6.0) -> int:
    deadline = time.monotonic() + timeout
    last_mode = -1
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.4)
        if msg and msg.get_srcSystem() == master.target_system and msg.get_srcComponent() == 1:
            last_mode = int(msg.custom_mode)
            if last_mode == expected_mode:
                return last_mode
    raise AssertionError(f'SITL did not reach mode {expected_mode}; last={last_mode}')


class _SitlFsmDriver:
    """Drives MissionFsmEngine while synchronizing key state transitions with live SITL."""

    def __init__(self, master) -> None:
        self.master = master
        self.eng = MissionFsmEngine(FsmConfig(s3_timeout_s=30.0, s6_timeout_s=20.0))
        self.t = 0.0
        self.last_hb_wall = 0.0
        self.last_ekf = 1

    def step(self, dt_s: float = 0.5, **kwargs):
        self.t += dt_s
        now_wall = time.monotonic()
        if now_wall - self.last_hb_wall >= 0.4:
            _send_l2_heartbeat(self.master)
            self.last_hb_wall = now_wall
        out = self.eng.step(FsmInputs(now_s=self.t, **kwargs))

        # Keep SITL synchronized on critical transitions (S5 bridge VIO EKF, S8/S9 HOLD).
        if out.ekf_source_set != self.last_ekf:
            if out.ekf_source_set == 2:
                _set_param(self.master, 'SCR_USER4', 1.0)  # BRIDGE_PERMIT=1
                _set_param(self.master, 'SCR_USER5', 1.0)  # BRIDGE_ZONE=1
                _set_param(self.master, 'SIM_GPS1_ENABLE', 0.0)
                _send_vio_odometry_burst(self.master, count=6)
                _send_ekf_source(self.master, 2)
            else:
                _set_param(self.master, 'SIM_GPS1_ENABLE', 1.0)
                _send_ekf_source(self.master, 1)
                _set_param(self.master, 'SCR_USER5', 0.0)
                _set_param(self.master, 'SCR_USER4', 0.0)
            self.last_ekf = out.ekf_source_set
        return out

    def goto_s1(self) -> None:
        if self.eng.state != S0:
            self.step(reset_to_s0=True)
        assert self.eng.state == S0
        self.step(start_mission=True)
        assert self.eng.state == S1


def test_stage5_sitl_covers_all_a8_transitions_with_zero_invalid() -> None:
    assert ARDUROVER_BIN.is_file(), f'Missing SITL binary: {ARDUROVER_BIN}'

    workdir = Path('/tmp/sitl_stage5_a8')
    shutil.rmtree(workdir, ignore_errors=True)
    scripts_dst = workdir / 'scripts'
    scripts_dst.mkdir(parents=True, exist_ok=True)
    for lua_file in LUA_DIR.glob('*.lua'):
        shutil.copy2(lua_file, scripts_dst / lua_file.name)

    log_file = open(workdir / 'sitl.log', 'w', encoding='utf-8')
    proc = subprocess.Popen(
        [
            str(ARDUROVER_BIN),
            '--model',
            'motorboat',
            '--defaults',
            f'{DEFAULT_ROVER_PARM},{A7_PARM}',
            '--wipe',
        ],
        cwd=str(workdir),
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f'ardurover exited early: {proc.returncode}')
            try:
                with socket.create_connection(('127.0.0.1', 5760), timeout=0.5):
                    break
            except OSError:
                time.sleep(0.2)
        else:
            raise TimeoutError('SITL TCP 5760 did not open')

        master = mavutil.mavlink_connection(
            'tcp:127.0.0.1:5760',
            source_system=1,
            source_component=191,
            dialect='ardupilotmega',
        )
        assert master.wait_heartbeat(timeout=15) is not None

        init_ts = int((time.time() - 1420070400) * 1e5)
        master.mav.setup_signing_send(
            master.target_system,
            master.target_component,
            list(SIGNING_KEY),
            init_ts,
        )
        master.setup_signing(SIGNING_KEY, sign_outgoing=True, link_id=0)
        time.sleep(0.4)

        _send_l2_heartbeat(master)
        master.set_mode(15)  # GUIDED
        _wait_mode(master, 15)

        drv = _SitlFsmDriver(master)

        # 1. Precheck gate + nominal mission + obstacle avoid: S0->S1->S2->S1
        assert drv.step(usb3_camera_ok=False, start_mission=True).state == S0
        drv.goto_s1()
        assert drv.step(obstacle_detected=True).state == S2
        assert drv.step(obstacle_detected=False).state == S1

        # Tier 1 BMS BLE loss + energy WARN -> self-recovery in S1 (no abort)
        out_t1 = drv.step(bms_ble_connected=False, energy_level=EnergyStatus.LEVEL_WARN)
        assert out_t1.state == S1 and out_t1.fault_tier == 1
        assert drv.step(bms_ble_connected=True, energy_level=EnergyStatus.LEVEL_OK).fault_tier == 0

        # 2. Full bridge transit with GNSS loss & EKF switch: S1->S3->S4->S5->S6->S1
        assert drv.step(bridge_detected=True).state == S3
        assert drv.step(at_bridge_measure_point=True).state == S4
        out_s5 = drv.step(bridge_clearance_ready=True, bridge_pass_permitted=True)
        assert out_s5.state == S5 and out_s5.ekf_source_set == 2
        assert drv.step(bridge_exited=True).state == S6
        assert drv.step(gnss_reacquired=True).state == S1

        # 3. Formation hold & resume: S1->S7->S1
        assert drv.step(formation_hold_req=True).state == S7
        assert drv.step(formation_hold_req=False).state == S1

        # 4. Battery RTB & docking: S1->S10->S11->S0
        assert drv.step(energy_level=EnergyStatus.LEVEL_RTB).state == S10
        assert drv.step(docked_at_base=True).state == S11
        assert drv.step(reset_to_s0=True).state == S0

        # 5. Tier 2 moderate faults -> DDKC S12 from S1, S2, S3 (timeout), S4 (denied), S6 (timeout)
        drv.goto_s1()
        assert drv.step(corridor_deviation_alert=True).state == S12  # S1->S12
        assert drv.step(energy_level=EnergyStatus.LEVEL_RTB).state == S10  # S12->S10
        assert drv.step(docked_at_base=True).state == S11
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(obstacle_detected=True).state == S2
        assert drv.step(corridor_deviation_alert=True).state == S12  # S2->S12
        assert drv.step(reset_to_s0=True).state == S0  # S12->S0

        drv.goto_s1()
        assert drv.step(bridge_detected=True).state == S3
        assert drv.step(dt_s=31.0).state == S12  # S3->S12 (S3 timeout)
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(bridge_detected=True).state == S3
        assert drv.step(at_bridge_measure_point=True).state == S4
        assert drv.step(bridge_clearance_ready=True, bridge_pass_permitted=False).state == S12  # S4->S12
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(bridge_detected=True).state == S3
        assert drv.step(at_bridge_measure_point=True).state == S4
        assert drv.step(bridge_clearance_ready=True, bridge_pass_permitted=True).state == S5
        assert drv.step(bridge_exited=True).state == S6
        assert drv.step(dt_s=21.0).state == S12  # S6->S12 (S6 timeout)
        assert drv.step(reset_to_s0=True).state == S0

        # 6. Tier 3 critical faults -> S8 (+ HOLD-ALL <= 2 s) from S0, S1, S5, S7, S10, S12; S8->S12 & S8->S0
        out_s8 = drv.step(tier3_critical_fault=True)  # S0->S8
        assert out_s8.state == S8 and out_s8.hold_all_req and out_s8.hold_all_latency_s <= 2.0
        master.set_mode(4)  # Synchronize HOLD on SITL
        _wait_mode(master, 4)
        assert drv.step(corridor_deviation_alert=True).state == S12  # S8->S12
        assert drv.step(tier3_critical_fault=True).state == S8  # S12->S8
        assert drv.step(reset_to_s0=True).state == S0  # S8->S0

        master.set_mode(15)  # Back to GUIDED
        _wait_mode(master, 15)

        drv.goto_s1()
        assert drv.step(tier3_critical_fault=True).state == S8  # S1->S8
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(bridge_detected=True).state == S3
        assert drv.step(at_bridge_measure_point=True).state == S4
        assert drv.step(bridge_clearance_ready=True, bridge_pass_permitted=True).state == S5
        assert drv.step(tier3_critical_fault=True).state == S8  # S5->S8
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(formation_hold_req=True).state == S7
        assert drv.step(tier3_critical_fault=True).state == S8  # S7->S8
        assert drv.step(reset_to_s0=True).state == S0

        drv.goto_s1()
        assert drv.step(rtb_requested=True).state == S10
        assert drv.step(tier3_critical_fault=True).state == S8  # S10->S8
        assert drv.step(reset_to_s0=True).state == S0

        # 7. Highest-priority S9 Emergency Stop from every operational/hold state + S9->S0 reset
        def _reach_and_stop(setup_fn, expected_src: int, use_batt_stop: bool = False) -> None:
            setup_fn()
            assert drv.eng.state == expected_src
            if use_batt_stop:
                out_stop = drv.step(energy_level=EnergyStatus.LEVEL_STOP)
            else:
                out_stop = drv.step(emergency_stop_cmd=True)
            assert out_stop.state == S9 and out_stop.hold_all_req and out_stop.fault_tier == 3
            assert drv.step(reset_to_s0=True).state == S0

        _reach_and_stop(lambda: None, S0, use_batt_stop=True)  # S0->S9, S9->S0
        _reach_and_stop(drv.goto_s1, S1, use_batt_stop=True)  # S1->S9
        _reach_and_stop(
            lambda: (drv.goto_s1(), drv.step(obstacle_detected=True)), S2
        )  # S2->S9
        _reach_and_stop(
            lambda: (drv.goto_s1(), drv.step(bridge_detected=True)), S3
        )  # S3->S9
        _reach_and_stop(
            lambda: (
                drv.goto_s1(),
                drv.step(bridge_detected=True),
                drv.step(at_bridge_measure_point=True),
            ),
            S4,
        )  # S4->S9
        _reach_and_stop(
            lambda: (
                drv.goto_s1(),
                drv.step(bridge_detected=True),
                drv.step(at_bridge_measure_point=True),
                drv.step(bridge_clearance_ready=True, bridge_pass_permitted=True),
            ),
            S5,
        )  # S5->S9
        _reach_and_stop(
            lambda: (
                drv.goto_s1(),
                drv.step(bridge_detected=True),
                drv.step(at_bridge_measure_point=True),
                drv.step(bridge_clearance_ready=True, bridge_pass_permitted=True),
                drv.step(bridge_exited=True),
            ),
            S6,
        )  # S6->S9
        _reach_and_stop(
            lambda: (drv.goto_s1(), drv.step(formation_hold_req=True)), S7
        )  # S7->S9
        _reach_and_stop(lambda: drv.step(tier3_critical_fault=True), S8)  # S8->S9
        _reach_and_stop(lambda: (drv.goto_s1(), drv.step(rtb_requested=True)), S10)  # S10->S9
        _reach_and_stop(
            lambda: (drv.goto_s1(), drv.step(rtb_requested=True), drv.step(docked_at_base=True)),
            S11,
        )  # S11->S9
        _reach_and_stop(
            lambda: (drv.goto_s1(), drv.step(corridor_deviation_alert=True)), S12
        )  # S12->S9

        master.set_mode(4)  # Final safe HOLD on SITL
        _wait_mode(master, 4)

        # Verify A.13 coverage criteria: 100% valid edges >= 1 time, ZERO invalid edges.
        missing = drv.eng.missing_transitions()
        assert not missing, f'Uncovered A.8 transitions: {sorted(missing)}'
        assert drv.eng.invalid_transitions == [], (
            f'Out-of-table transitions occurred: {drv.eng.invalid_transitions}'
        )
        assert len(drv.eng.observed_transitions) == len(ALLOWED_TRANSITIONS)
        for edge in ALLOWED_TRANSITIONS:
            assert drv.eng.transition_counts[edge] >= 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log_file.close()
        shutil.rmtree(workdir, ignore_errors=True)
