"""A.11 / TR-4.1 end-to-end latency statistics and HIL P99 measurement tool."""

import argparse
import json
import math
import time
from typing import Any, Dict, List, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from nenuphar_nodes.mavlink_bridge_node import E2E_LATENCY_TOPIC
from nenuphar_nodes.qos import SENSOR_QOS

# Fixed hardware budget components from SPEC_PHU_LUC_A.md §8 (Table A.11)
A11_FRAME_WAIT_MAX_MS = 67.0      # 15 Hz frame interval (TBD-21)
A11_HALF_EXPOSURE_MAX_MS = 5.0
A11_OAKD_CONFIG_A_MS = 21.0       # Config A (400P) on-device stereo/median
A11_L1_AUTOPILOT_MS = 20.0        # ArduPilot Rover 50 Hz loop
A11_DSHOT_ESC_MS = 1.0            # Bidirectional DShot < 1 ms
TR41_BUDGET_MS = 200.0
L2_PIPELINE_BUDGET_MS = 65.0      # perception_geom <= 50 ms + fsm/bridge <= 15 ms


def percentile(values: Sequence[float], pct: float) -> float:
    """Compute linear-interpolated percentile (0..100) without NumPy dependency."""
    if not values:
        raise ValueError('Cannot compute percentile of empty sample list')
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def compute_latency_stats(samples_ms: Sequence[float]) -> Dict[str, float]:
    """Return count, mean, p50, p95, p99, and max latency in milliseconds."""
    if not samples_ms:
        raise ValueError('No latency samples collected')
    vals = [float(x) for x in samples_ms]
    return {
        'count': float(len(vals)),
        'mean_ms': sum(vals) / len(vals),
        'p50_ms': percentile(vals, 50.0),
        'p95_ms': percentile(vals, 95.0),
        'p99_ms': percentile(vals, 99.0),
        'max_ms': max(vals),
    }


def evaluate_a11_budget(
    l2_pipeline_p99_ms: float,
    l1_roundtrip_ms: float = A11_L1_AUTOPILOT_MS,
    include_camera_hw_worst_case: bool = True,
    l2_pipeline_max_ms: float = 0.0,
) -> Dict[str, Any]:
    """Evaluate measured L2 P99 & max latency against TR-4.1 (<= 200 ms) Config A budget."""
    hw_ms = (
        A11_FRAME_WAIT_MAX_MS + A11_HALF_EXPOSURE_MAX_MS + A11_OAKD_CONFIG_A_MS
        if include_camera_hw_worst_case
        else 0.0
    )
    total_worst_case_ms = hw_ms + float(l2_pipeline_p99_ms) + float(l1_roundtrip_ms) + A11_DSHOT_ESC_MS
    max_e2e_ms = hw_ms + float(l2_pipeline_max_ms) + float(l1_roundtrip_ms) + A11_DSHOT_ESC_MS
    return {
        'l2_pipeline_p99_ms': float(l2_pipeline_p99_ms),
        'l2_pipeline_max_ms': float(l2_pipeline_max_ms),
        'l2_budget_ms': L2_PIPELINE_BUDGET_MS,
        'l2_within_budget': float(l2_pipeline_p99_ms) <= L2_PIPELINE_BUDGET_MS,
        'camera_hw_worst_case_ms': hw_ms,
        'l1_and_dshot_ms': float(l1_roundtrip_ms) + A11_DSHOT_ESC_MS,
        'total_e2e_worst_case_ms': total_worst_case_ms,
        'total_e2e_max_ms': max_e2e_ms,
        'tr41_budget_ms': TR41_BUDGET_MS,
        'tr41_pass': total_worst_case_ms <= TR41_BUDGET_MS,
        'max_within_tr41': max_e2e_ms <= TR41_BUDGET_MS,
    }


def main(args=None) -> None:
    """CLI tool: subscribe to /mavlink/e2e_latency_ms and print A.11 P99 JSON report."""
    parser = argparse.ArgumentParser(description='NENUPHARES A.11 P99 latency probe')
    parser.add_argument('--samples', type=int, default=50, help='Target sample count')
    parser.add_argument('--timeout', type=float, default=15.0, help='Max listen time (s)')
    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = Node('latency_probe')
    samples: List[float] = []
    node.create_subscription(
        Float64, E2E_LATENCY_TOPIC, lambda m: samples.append(float(m.data)), SENSOR_QOS
    )
    try:
        deadline = time.monotonic() + parsed.timeout
        while len(samples) < parsed.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not samples:
            raise SystemExit('ERROR: No samples received on /mavlink/e2e_latency_ms')
        stats = compute_latency_stats(samples)
        report = {
            'stats': stats,
            'a11_budget': evaluate_a11_budget(
                stats['p99_ms'], l2_pipeline_max_ms=stats['max_ms']
            ),
        }
        print(json.dumps(report, indent=2))
        if not report['a11_budget']['tr41_pass']:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
