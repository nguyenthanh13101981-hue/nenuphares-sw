"""Stage 6 SITL verification: A.11 P99 latency (<= 200 ms) and 4-fault injection suite.

Exercises all Stage 6 deliverables against live ROS 2 nodes, ArduPilot Boat SITL, and L0 C8:
  1. End-to-end timestamped latency probe (camera_driver -> perception_geom -> mission_fsm ->
     mavlink_bridge -> SITL L1) verifying P99 within TR-4.1 <= 200 ms Config A budget.
  2. Fault Injection #1: USB 3 camera disconnect -> S0 startup strictly rejected.
  3. Fault Injection #2: BMS Bluetooth LE loss (TBD-23) -> warn only, never stops raft.
  4. Fault Injection #3: L2 heartbeat loss -> ArduPilot Lua watchdog trips HOLD mode.
  5. Fault Injection #4: LoRa 12-byte tamper/replay/drop -> L0 C8 safety verifier rejects.
"""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

from pymavlink import mavutil
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

os.environ['MAVLINK20'] = '1'

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'l2_ros2_ws/src/nenuphar_nodes'))

from nenuphar_msgs.msg import EnergyStatus, MissionState  # noqa: E402
from nenuphar_nodes.camera_driver_node import CameraDriverNode  # noqa: E402
from nenuphar_nodes.energy_node import EnergyNode  # noqa: E402
from nenuphar_nodes.latency_probe import (  # noqa: E402
    compute_latency_stats,
    evaluate_a11_budget,
)
from nenuphar_nodes.mavlink_bridge_node import MavlinkBridgeNode  # noqa: E402
from nenuphar_nodes.mission_fsm_node import MissionFsmNode  # noqa: E402
from nenuphar_nodes.perception_geom_node import PerceptionGeomNode  # noqa: E402
from nenuphar_nodes.system_monitor_node import SystemMonitorNode  # noqa: E402

ARDUROVER_BIN = Path.home() / 'sim/ardupilot/build/sitl/bin/ardurover'
DEFAULT_ROVER_PARM = Path.home() / 'sim/ardupilot/Tools/autotest/default_params/rover.parm'
A7_PARM = REPO_ROOT / 'l1_ardupilot/params/sitl_boat_a7.parm'
LUA_DIR = REPO_ROOT / 'l1_ardupilot/lua'
C8_TEST_BIN = REPO_ROOT / 'l0_c8_pico2/build/test_c8_safety'
SIGNING_KEY_HEX = '4e454e5550484152' * 4


def _start_sitl(workdir: Path):
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
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            log_file.close()
            raise RuntimeError(f'ardurover exited early: {proc.returncode}')
        try:
            with socket.create_connection(('127.0.0.1', 5760), timeout=0.5):
                return proc, log_file
        except OSError:
            time.sleep(0.2)
    proc.kill()
    log_file.close()
    raise TimeoutError('SITL TCP 5760 did not open')


def _stop_sitl(proc, log_file, workdir: Path) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
    log_file.close()
    shutil.rmtree(workdir, ignore_errors=True)


def test_stage6_a11_p99_latency_and_four_fault_injections() -> None:
    assert ARDUROVER_BIN.is_file(), f'Missing SITL binary: {ARDUROVER_BIN}'
    assert C8_TEST_BIN.is_file(), f'Missing L0 C8 test binary: {C8_TEST_BIN}'

    workdir = Path('/tmp/sitl_stage6_hil')
    proc, log_file = _start_sitl(workdir)
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        # 1. Establish signed MAVLink 2 channel via MavlinkBridgeNode to live SITL.
        cam = CameraDriverNode(
            parameter_overrides=[Parameter('enable_stub_stream', value=True)]
        )
        geom = PerceptionGeomNode()
        fsm = MissionFsmNode()
        energy = EnergyNode(
            parameter_overrides=[
                Parameter('bms_ble_connected', value=False),
                Parameter('stub_soc_pct', value=75.0),
                Parameter('stub_voltage_v', value=13.2),
            ]
        )
        sysmon = SystemMonitorNode(
            parameter_overrides=[
                Parameter('usb3_check_mode', value='param'),
                Parameter('usb3_camera_ok', value=False),
                Parameter('publish_period_s', value=0.25),
            ]
        )
        bridge = MavlinkBridgeNode(
            parameter_overrides=[
                Parameter('mavlink_url', value='tcp:127.0.0.1:5760'),
            ]
        )
        nodes = [cam, geom, fsm, energy, sysmon, bridge]
        for n in nodes:
            executor.add_node(n)

        # Complete SITL signing handshake, then activate strict MAVLink 2 signing on bridge.
        assert bridge._mav is not None
        assert bridge._mav.wait_heartbeat(timeout=15) is not None
        key_bytes = bytes.fromhex(SIGNING_KEY_HEX)
        init_ts = int((time.time() - 1420070400) * 1e5)
        bridge._mav.mav.setup_signing_send(
            bridge._mav.target_system,
            bridge._mav.target_component,
            list(key_bytes),
            init_ts,
        )
        bridge.enable_signing(SIGNING_KEY_HEX)
        assert bridge.signing_enabled is True
        time.sleep(0.3)

        # Spin >= 2.5 s to collect >= 30 A.11 timestamped latency samples (15 Hz stub stream).
        fsm.inputs.start_mission = True
        deadline = time.monotonic() + 3.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if len(bridge.e2e_latencies_ms) >= 35 and energy.ble_warn_count >= 1:
                break

        # Verify A.11 P99 latency <= 200 ms TR-4.1 budget (Config A 400P).
        assert len(bridge.e2e_latencies_ms) >= 30, (
            f'Expected >= 30 latency samples, got {len(bridge.e2e_latencies_ms)}'
        )
        stats = compute_latency_stats(bridge.e2e_latencies_ms)
        budget = evaluate_a11_budget(stats['p99_ms'])
        print(
            f'\n[A.11 P99] count={int(stats["count"])} p50={stats["p50_ms"]:.2f}ms '
            f'p95={stats["p95_ms"]:.2f}ms p99={stats["p99_ms"]:.2f}ms '
            f'e2e_worst={budget["total_e2e_worst_case_ms"]:.2f}ms '
            f'tr41_pass={budget["tr41_pass"]}'
        )
        assert budget['l2_within_budget'] is True
        assert budget['tr41_pass'] is True

        # Fault Injection #1: USB 3 camera disconnect -> FSM MUST remain in S0.
        assert fsm.state == MissionState.STATE_S0

        # Fault Injection #2: BMS Bluetooth LE disconnect (TBD-23) -> warn only, never STOP.
        assert energy.ble_warn_count >= 1
        assert fsm.inputs.bms_ble_connected is False
        assert fsm.inputs.energy_level == EnergyStatus.LEVEL_OK
        # Restore USB 3 & all_nodes_alive -> FSM enters S1 with Tier 1 minor warning only.
        sysmon.set_parameters([Parameter('usb3_camera_ok', value=True)])
        fsm.inputs.usb3_camera_ok = True
        fsm.inputs.all_nodes_alive = True
        fsm._step_and_publish_on_change()
        assert fsm.state == MissionState.STATE_S1
        assert fsm.last_output.fault_tier == 1

        # Fault Injection #3: L2 heartbeat loss -> ArduPilot Lua watchdog forces HOLD (mode 4).
        master = bridge._mav
        master.set_mode(15)  # GUIDED
        mode_deadline = time.monotonic() + 5.0
        while time.monotonic() < mode_deadline:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.3)
            if hb and hb.get_srcSystem() == master.target_system and hb.get_srcComponent() == 1:
                if int(hb.custom_mode) == 15:
                    break
        # Stop spinning ROS timers (no more L2 heartbeats sent) and observe Lua watchdog trip >3s.
        wd_tripped = False
        hold_mode = False
        wd_deadline = time.monotonic() + 7.0
        while time.monotonic() < wd_deadline and not (wd_tripped and hold_mode):
            msg = master.recv_match(
                type=['NAMED_VALUE_FLOAT', 'HEARTBEAT'], blocking=True, timeout=0.4
            )
            if msg is None:
                continue
            if msg.get_type() == 'NAMED_VALUE_FLOAT' and msg.name.rstrip('\x00') == 'L2_WD_TRIP':
                if float(msg.value) == 1.0:
                    wd_tripped = True
            elif (
                msg.get_type() == 'HEARTBEAT'
                and msg.get_srcSystem() == master.target_system
                and msg.get_srcComponent() == 1
                and int(msg.custom_mode) == 4
            ):
                hold_mode = True
        assert wd_tripped, 'Expected L2_WD_TRIP=1.0 after stopping Pi 5 heartbeat >3s'
        assert hold_mode, 'Expected ArduPilot Rover SITL to switch to HOLD (mode 4)'

        # Fault Injection #4: LoRa 12-byte tamper/replay/drop on L0 C8 core.
        c8_res = subprocess.run(
            [
                str(C8_TEST_BIN),
                '--gtest_filter=C8SafetyTest.Lora12ByteHmacVerificationAndAntiReplay:'
                'C8SafetyTest.KeepaliveLossThreeSecondsTriggersHoldReqAndRecovers',
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert c8_res.returncode == 0, c8_res.stdout + c8_res.stderr
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_sitl(proc, log_file, workdir)
