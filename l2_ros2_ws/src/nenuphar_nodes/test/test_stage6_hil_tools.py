"""Stage 6 unit tests: A.11 latency probe math and blackbox rosbag2 analyzer."""

from pathlib import Path
import sys
import tempfile

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py

from nenuphar_msgs.msg import MissionState, SystemHealth
from nenuphar_nodes.latency_probe import (
    compute_latency_stats,
    evaluate_a11_budget,
)

REPO_ROOT = Path('/home/nenuphar/nenuphares-sw')
sys.path.insert(0, str(REPO_ROOT / 'tools'))
from analyze_blackbox_bag import analyze_bag  # noqa: E402


def test_latency_stats_and_a11_budget_checker() -> None:
    samples = [float(i) for i in range(1, 101)]  # 1..100 ms
    stats = compute_latency_stats(samples)
    assert stats['count'] == 100.0
    assert abs(stats['p50_ms'] - 50.5) < 1e-6
    assert abs(stats['p95_ms'] - 95.05) < 1e-6
    assert abs(stats['p99_ms'] - 99.01) < 1e-6

    # Config A budget: 67 + 5 + 21 + L2_P99 + 20 + 1 <= 200 ms -> L2 <= 86 ms (budget 65 ms).
    ok_res = evaluate_a11_budget(l2_pipeline_p99_ms=42.0)
    assert ok_res['l2_within_budget'] is True
    assert ok_res['tr41_pass'] is True
    assert ok_res['total_e2e_worst_case_ms'] == 156.0

    # Config B / overloaded Pi 5 exceeding 200 ms fails TR-4.1.
    fail_res = evaluate_a11_budget(l2_pipeline_p99_ms=95.0)
    assert fail_res['l2_within_budget'] is False
    assert fail_res['tr41_pass'] is False


def test_analyze_blackbox_rosbag2_normal_1hz_and_event_10hz() -> None:
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='nenuphar_bag_test_') as tmpdir:
            bag_dir = Path(tmpdir) / 'blackbox_sample'
            writer = rosbag2_py.SequentialWriter()
            storage_opts = rosbag2_py.StorageOptions(
                uri=str(bag_dir), storage_id='sqlite3'
            )
            conv_opts = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr', output_serialization_format='cdr'
            )
            writer.open(storage_opts, conv_opts)

            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=0,
                    name='/fsm/state',
                    type='nenuphar_msgs/msg/MissionState',
                    serialization_format='cdr',
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=1,
                    name='/system/health',
                    type='nenuphar_msgs/msg/SystemHealth',
                    serialization_format='cdr',
                )
            )

            base_ns = 1_700_000_000_000_000_000
            # 5 seconds of 1 Hz normal operation (S1, fault_tier=0).
            for i in range(6):
                ts = base_ns + i * 1_000_000_000
                st = MissionState()
                st.state = MissionState.STATE_S1
                st.fault_tier = 0
                writer.write('/fsm/state', serialize_message(st), ts)

                hl = SystemHealth()
                hl.usb3_camera_ok = True
                hl.all_nodes_alive = True
                writer.write('/system/health', serialize_message(hl), ts)

            # 2 seconds of 10 Hz bridge measurement event (S4).
            event_start = base_ns + 6_000_000_000
            for j in range(21):
                ts = event_start + j * 100_000_000  # 10 Hz
                st = MissionState()
                st.state = MissionState.STATE_S4
                st.fault_tier = 0
                writer.write('/fsm/state', serialize_message(st), ts)

            del writer

            report = analyze_bag(bag_dir)
            assert report['storage_id'] == 'sqlite3'
            assert report['fits_256gb_nvme_24h'] is True
            assert report['topics']['/fsm/state']['count'] == 27
            assert report['topics']['/system/health']['count'] == 6
            assert 0.9 <= report['fsm_rates']['normal_rate_hz'] <= 1.1
            assert report['fsm_rates']['event_or_s4_rate_hz'] >= 9.5
            assert report['fsm_rates']['event_rate_meets_10hz'] is True
    finally:
        if rclpy.ok():
            rclpy.shutdown()
