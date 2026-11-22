"""MAVLink 2 bridge skeleton forwarding system_monitor heartbeat to L1/Pixhawk."""

from typing import Optional

from nav_msgs.msg import Odometry
from nenuphar_msgs.msg import (
    BridgeClearance,
    EnergyStatus,
    GuidedSetpoint,
    MissionState,
    SystemHealth,
)
from std_msgs.msg import UInt64

from nenuphar_nodes.base_node import (
    MAVLINK_HEARTBEAT_TX_TOPIC,
    MonitoredNode,
    run_node_main,
)
from nenuphar_nodes.qos import CMD_QOS, SENSOR_QOS


class MavlinkBridgeNode(MonitoredNode):
    """Bridges /system/health heartbeat from system_monitor to MAVLink 2 TX."""

    def __init__(self) -> None:
        super().__init__('mavlink_bridge')
        self.declare_parameter('mavlink_url', '')

        self._mav = None
        mav_url = str(self.get_parameter('mavlink_url').value).strip()
        if mav_url:
            from pymavlink import mavutil

            self._mav = mavutil.mavlink_connection(
                mav_url, source_system=1, source_component=191
            )

        self.tx_heartbeat_count = 0
        self.last_health: Optional[SystemHealth] = None

        self.health_sub = self.create_subscription(
            SystemHealth, '/system/health', self._on_health, CMD_QOS
        )
        self.fsm_state_sub = self.create_subscription(
            MissionState, '/fsm/state', self._on_fsm_state, CMD_QOS
        )
        self.setpoint_sub = self.create_subscription(
            GuidedSetpoint, '/fsm/cmd_setpoint', self._on_setpoint, CMD_QOS
        )
        self.clearance_sub = self.create_subscription(
            BridgeClearance, '/clearance/bridge_result', self._on_clearance, CMD_QOS
        )
        self.energy_sub = self.create_subscription(
            EnergyStatus, '/energy/status', self._on_energy, CMD_QOS
        )
        self.vio_sub = self.create_subscription(
            Odometry, '/vio/odometry', self._on_vio, SENSOR_QOS
        )

        self.hb_tx_pub = self.create_publisher(
            UInt64, MAVLINK_HEARTBEAT_TX_TOPIC, CMD_QOS
        )

    def _on_health(self, msg: SystemHealth) -> None:
        self.last_health = msg
        self.tx_heartbeat_count += 1

        if self._mav is not None:
            from pymavlink import mavutil

            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
                if msg.usb3_camera_ok
                else mavutil.mavlink.MAV_STATE_CRITICAL,
            )

        tx_msg = UInt64()
        tx_msg.data = self.tx_heartbeat_count
        self.hb_tx_pub.publish(tx_msg)

    def _on_fsm_state(self, _msg: MissionState) -> None:
        pass

    def _on_setpoint(self, _msg: GuidedSetpoint) -> None:
        pass

    def _on_clearance(self, _msg: BridgeClearance) -> None:
        pass

    def _on_energy(self, _msg: EnergyStatus) -> None:
        pass

    def _on_vio(self, _msg: Odometry) -> None:
        pass


def main(args=None) -> None:
    run_node_main(MavlinkBridgeNode, args)
