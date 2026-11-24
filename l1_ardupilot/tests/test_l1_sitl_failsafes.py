"""Stage 4 SITL verification: Boat A.7 params, Lua failsafes M1-M4, signing, ODOMETRY & EKF."""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import time

import pytest
from pymavlink import mavutil

os.environ['MAVLINK20'] = '1'

REPO_ROOT = Path(__file__).resolve().parents[2]
ARDUROVER_BIN = Path.home() / 'sim/ardupilot/build/sitl/bin/ardurover'
DEFAULT_ROVER_PARM = Path.home() / 'sim/ardupilot/Tools/autotest/default_params/rover.parm'
A7_PARM = REPO_ROOT / 'l1_ardupilot/params/sitl_boat_a7.parm'
LUA_DIR = REPO_ROOT / 'l1_ardupilot/lua'

SIGNING_KEY = bytes.fromhex('4e454e5550484152' * 4)  # 32-byte shared MAVLink 2 key
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
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip('\x00') == name:
            return float(msg.param_value)
    raise TimeoutError(f'Timed out setting param {name}={value}')


def _get_param(master, name: str, timeout: float = 5.0) -> float:
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode('ascii'),
        -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip('\x00') == name:
            return float(msg.param_value)
    raise TimeoutError(f'Timed out reading param {name}')


def _wait_named_float(master, name: str, predicate, timeout: float = 8.0) -> float:
    deadline = time.monotonic() + timeout
    last_val = None
    while time.monotonic() < deadline:
        msg = master.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=0.5)
        if msg and msg.name.rstrip('\x00') == name:
            last_val = float(msg.value)
            if predicate(last_val):
                return last_val
    raise AssertionError(f'NAMED_VALUE_FLOAT {name} did not match predicate; last={last_val}')


def _send_l2_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def _wait_mode(master, expected_mode: int, timeout: float = 8.0) -> int:
    deadline = time.monotonic() + timeout
    last_mode = -1
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg and msg.get_srcSystem() == master.target_system and msg.get_srcComponent() == 1:
            last_mode = int(msg.custom_mode)
            if last_mode == expected_mode:
                return last_mode
    raise AssertionError(f'Vehicle did not reach mode {expected_mode}; last={last_mode}')


def test_stage4_sitl_boat_lua_failsafes_signing_and_ekf() -> None:
    assert ARDUROVER_BIN.is_file(), f'Missing SITL binary: {ARDUROVER_BIN}'

    workdir = Path('/tmp/sitl_stage4')
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

        # Connect as L2 onboard computer (compid=191) using MAVLink 2.
        master = mavutil.mavlink_connection(
            'tcp:127.0.0.1:5760',
            source_system=1,
            source_component=191,
            dialect='ardupilotmega',
        )
        hb = master.wait_heartbeat(timeout=15)
        assert hb is not None

        # Enable MAVLink 2 signing on both SITL and client.
        init_ts = int((time.time() - 1420070400) * 1e5)
        master.mav.setup_signing_send(
            master.target_system,
            master.target_component,
            list(SIGNING_KEY),
            init_ts,
        )
        master.setup_signing(SIGNING_KEY, sign_outgoing=True, link_id=0)
        time.sleep(0.5)

        # Verify A.7 baseline params & active failsafes/geofence.
        assert _get_param(master, 'FRAME_CLASS') == 2.0
        assert _get_param(master, 'SCHED_LOOP_RATE') == 50.0
        assert _get_param(master, 'FS_TIMEOUT') >= 3.0
        assert _get_param(master, 'FS_ACTION') == 2.0
        assert _get_param(master, 'FS_GCS_ENABLE') == 1.0
        assert _get_param(master, 'BATT_MONITOR') == 4.0
        assert _get_param(master, 'BATT_FS_LOW_ACT') == 1.0
        assert _get_param(master, 'BATT_FS_CRT_ACT') == 2.0
        assert _get_param(master, 'FENCE_ENABLE') == 1.0
        assert _get_param(master, 'FENCE_ACTION') == 2.0
        assert abs(_get_param(master, 'SIM_TIDE_SPEED') - 0.10) < 1e-3
        assert abs(_get_param(master, 'SIM_WIND_SPD') - 0.10) < 1e-3

        # Wait until Lua scripts are running and emitting MIX_MODE=1.0.
        assert _wait_named_float(master, 'MIX_MODE', lambda v: v == 1.0, timeout=12.0) == 1.0
        assert _wait_named_float(master, 'CURR_LIM_A', lambda v: v == 70.0) == 70.0

        # Scenario 1: Single thruster failure M1 -> M2 -> M4 -> M1.
        _set_param(master, 'SCR_USER1', 1.0)
        assert _wait_named_float(master, 'MIX_MODE', lambda v: v == 2.0) == 2.0
        assert _wait_named_float(master, 'CURR_LIM_A', lambda v: v == 40.0) == 40.0

        _set_param(master, 'SCR_USER1', 3.0)
        assert _wait_named_float(master, 'MIX_MODE', lambda v: v == 4.0) == 4.0

        # Scenario 2: Overcurrent limit 40 A (M4/M2) and 70 A (M1).
        _set_param(master, 'SCR_USER2', 50.0)
        scale_backup = _wait_named_float(master, 'CURR_SCALE', lambda v: v <= 0.82)
        assert 0.75 <= scale_backup <= 0.82

        _set_param(master, 'SCR_USER1', 0.0)
        assert _wait_named_float(master, 'MIX_MODE', lambda v: v == 1.0) == 1.0
        _set_param(master, 'SCR_USER2', 84.0)
        scale_normal = _wait_named_float(master, 'CURR_SCALE', lambda v: v <= 0.86)
        assert 0.80 <= scale_normal <= 0.86

        _set_param(master, 'SCR_USER2', 0.0)
        assert _wait_named_float(master, 'CURR_SCALE', lambda v: abs(v - 1.0) < 1e-3) == 1.0

        # Scenario 3: Bridge permit lock, GCS timeout relaxation, and Pi 5 watchdog (>3s -> HOLD).
        _send_l2_heartbeat(master)
        master.set_mode(15)  # GUIDED
        _wait_mode(master, 15)

        # Enter BRIDGE_ZONE without BRIDGE_PERMIT -> must force HOLD (4) when in GUIDED.
        _set_param(master, 'SCR_USER4', 0.0)
        _set_param(master, 'SCR_USER5', 1.0)
        _send_l2_heartbeat(master)
        _wait_mode(master, 4)
        assert _wait_named_float(master, 'BRG_LOCK', lambda v: v == 1.0) == 1.0

        # Human operator override (MANUAL = 0) must NEVER be overridden by Lua bridge lock.
        master.set_mode(0)
        _wait_mode(master, 0)
        time.sleep(0.4)
        _wait_mode(master, 0)

        # Grant BRIDGE_PERMIT -> FS_GCS_TIMEOUT relaxes to 30 s, allow GUIDED.
        _set_param(master, 'SCR_USER4', 1.0)
        _send_l2_heartbeat(master)
        assert _wait_named_float(master, 'BRG_LOCK', lambda v: v == 0.0) == 0.0
        assert _get_param(master, 'FS_GCS_TIMEOUT') == 30.0

        master.set_mode(15)  # GUIDED
        _wait_mode(master, 15)

        # Leave BRIDGE_ZONE -> FS_GCS_TIMEOUT restores to 5 s.
        _set_param(master, 'SCR_USER5', 0.0)
        time.sleep(0.4)
        assert _get_param(master, 'FS_GCS_TIMEOUT') == 5.0

        # Stop sending L2 heartbeat for >3.0 s -> Lua watchdog switches to HOLD (4).
        assert _wait_named_float(master, 'L2_WD_TRIP', lambda v: v == 1.0, timeout=7.0) == 1.0
        _wait_mode(master, 4)

        # Scenario 4: GNSS loss under bridge + 25 Hz VIO ODOMETRY + EKF source switch to 2.
        _send_l2_heartbeat(master)
        _set_param(master, 'SIM_GPS1_ENABLE', 0.0)
        cov = [float('nan')] + [0.0] * 20
        for _ in range(10):
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

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            MAV_CMD_SET_EKF_SOURCE_SET,
            0,
            2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        ack_deadline = time.monotonic() + 5.0
        ack_result = None
        while time.monotonic() < ack_deadline:
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
            if ack and ack.command == MAV_CMD_SET_EKF_SOURCE_SET:
                ack_result = int(ack.result)
                break
        assert ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED, f'EKF switch ACK={ack_result}'
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log_file.close()
        shutil.rmtree(workdir, ignore_errors=True)
