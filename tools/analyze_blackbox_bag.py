#!/usr/bin/env python3
"""Analyze rosbag2 logs produced by the blackbox node (1 Hz normal / >= 10 Hz event or S4)."""

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Dict, List

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze_bag(bag_path: Path) -> Dict[str, Any]:
    """Read a rosbag2 directory and validate normal vs event/S4 recording rates & 24h storage."""
    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f'Bag path does not exist: {bag_path}')

    storage_id = 'mcap' if list(bag_path.glob('*.mcap')) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader.open(storage_opts, conv_opts)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topic_timestamps: Dict[str, List[int]] = {name: [] for name in topic_types}
    normal_ts_ns: List[int] = []
    event_ts_ns: List[int] = []

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        topic_timestamps.setdefault(topic, []).append(ts_ns)
        if topic == '/fsm/state' and topic in topic_types:
            msg_cls = get_message(topic_types[topic])
            msg = deserialize_message(data, msg_cls)
            if int(msg.state) == 4 or int(msg.fault_tier) >= 2:
                event_ts_ns.append(ts_ns)
            else:
                normal_ts_ns.append(ts_ns)

    def _rate_hz(ts_list: List[int]) -> float:
        if len(ts_list) < 2:
            return 0.0
        dt_s = (ts_list[-1] - ts_list[0]) / 1e9
        return (len(ts_list) - 1) / dt_s if dt_s > 0 else 0.0

    all_ts = [t for lst in topic_timestamps.values() for t in lst]
    duration_s = ((max(all_ts) - min(all_ts)) / 1e9) if len(all_ts) >= 2 else 0.0
    total_bytes = sum(p.stat().st_size for p in bag_path.rglob('*') if p.is_file())
    bytes_per_sec = (total_bytes / duration_s) if duration_s > 0 else 0.0
    projected_24h_gb = (bytes_per_sec * 86400.0) / (1024.0 ** 3)

    normal_hz = _rate_hz(normal_ts_ns)
    event_hz = _rate_hz(event_ts_ns)

    return {
        'bag_path': str(bag_path),
        'storage_id': storage_id,
        'duration_s': round(duration_s, 3),
        'total_bytes': total_bytes,
        'projected_24h_gb': round(projected_24h_gb, 3),
        'fits_256gb_nvme_24h': projected_24h_gb < 200.0,
        'topics': {
            name: {
                'type': topic_types[name],
                'count': len(ts),
                'avg_rate_hz': round(_rate_hz(ts), 2),
            }
            for name, ts in topic_timestamps.items()
        },
        'fsm_rates': {
            'normal_count': len(normal_ts_ns),
            'normal_rate_hz': round(normal_hz, 2),
            'event_or_s4_count': len(event_ts_ns),
            'event_or_s4_rate_hz': round(event_hz, 2),
            'event_rate_meets_10hz': event_hz >= 9.5 if len(event_ts_ns) >= 2 else False,
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Analyze NENUPHARES blackbox rosbag2 logs')
    parser.add_argument('bag_path', type=Path, help='Path to rosbag2 directory')
    args = parser.parse_args(argv)
    report = analyze_bag(args.bag_path)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
