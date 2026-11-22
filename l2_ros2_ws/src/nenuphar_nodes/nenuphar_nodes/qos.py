"""Shared QoS profiles per docs/INTERFACE_FREEZE_A14.md."""

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# High-rate sensor/perception streams: non-blocking latest-frame semantics.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Command, state, and health channels.
CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
