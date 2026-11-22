"""Energy monitor node (1 Hz, shunt primary + BLE BMS comparison per A.9/TBD-23)."""

from typing import Any, Dict

from nenuphar_msgs.msg import EnergyStatus

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_energy_config
from nenuphar_nodes.qos import CMD_QOS


def classify_energy_level(
    soc_pct: float, voltage_v: float, cfg: Dict[str, Any]
) -> int:
    """Classify energy state using shunt measurements (BLE loss never forces STOP)."""
    em = cfg['energy_management']
    if soc_pct <= float(em['stop_soc_pct']) or voltage_v <= float(
        em['l1_batt_crt_v_min']
    ):
        return EnergyStatus.LEVEL_STOP
    if soc_pct <= float(em['rtb_soc_pct']):
        return EnergyStatus.LEVEL_RTB
    if soc_pct <= float(em['warn_soc_pct']):
        return EnergyStatus.LEVEL_WARN
    return EnergyStatus.LEVEL_OK


class EnergyNode(MonitoredNode):
    """Publishes 1 Hz /energy/status from config/energy_A9.yaml."""

    def __init__(self) -> None:
        super().__init__('energy')
        self.cfg = load_energy_config()
        self.declare_parameter('bms_ble_connected', True)
        self.declare_parameter('stub_soc_pct', 100.0)
        self.declare_parameter('stub_voltage_v', 13.2)

        period = 1.0 / float(self.cfg['energy_management'].get('loop_rate_hz', 1.0))
        self.status_pub = self.create_publisher(
            EnergyStatus, '/energy/status', CMD_QOS
        )
        self.timer = self.create_timer(period, self._publish_status)

    def _publish_status(self) -> None:
        ble_ok = bool(self.get_parameter('bms_ble_connected').value)
        if not ble_ok:
            # TBD-23: loss of BLE only warns; never stop the raft.
            self.get_logger().warning(
                'BMS Bluetooth LE disconnected; continuing on primary shunt',
                throttle_duration_sec=10.0,
            )

        soc = float(self.get_parameter('stub_soc_pct').value)
        volt = float(self.get_parameter('stub_voltage_v').value)
        total_wh = float(self.cfg['energy_management']['total_capacity_wh'])

        msg = EnergyStatus()
        msg.stamp = self.now_msg()
        msg.soc_pct = soc
        msg.voltage_v = volt
        msg.current_a = 0.0
        msg.wh_remaining = total_wh * (soc / 100.0)
        msg.level = classify_energy_level(soc, volt, self.cfg)
        msg.bms_ble_connected = ble_ok
        self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(EnergyNode, args)
