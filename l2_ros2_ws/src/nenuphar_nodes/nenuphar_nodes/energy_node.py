"""Energy monitor node (1 Hz, shunt Coulomb integration + BLE BMS per A.9/TBD-23)."""

from typing import Any, Dict

from nenuphar_msgs.msg import EnergyStatus

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_energy_config
from nenuphar_nodes.qos import CMD_QOS


def classify_energy_level(
    soc_pct: float, voltage_v: float, cfg: Dict[str, Any]
) -> int:
    """Classify energy state using primary shunt (BLE loss never forces STOP)."""
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


def integrate_shunt_wh(
    wh_remaining: float,
    voltage_v: float,
    current_a: float,
    dt_s: float,
    total_capacity_wh: float,
) -> float:
    """Integrate primary shunt power over dt_s and clamp to [0, total_capacity_wh]."""
    consumed_wh = max(0.0, voltage_v * max(0.0, current_a) * max(0.0, dt_s) / 3600.0)
    return max(0.0, min(total_capacity_wh, wh_remaining - consumed_wh))


class EnergyNode(MonitoredNode):
    """Publishes 1 Hz /energy/status from config/energy_A9.yaml."""

    def __init__(self, **kwargs) -> None:
        super().__init__('energy', **kwargs)
        self.cfg = load_energy_config()
        em = self.cfg['energy_management']
        self.total_wh = float(em['total_capacity_wh'])

        self.declare_parameter('bms_ble_connected', True)
        self.declare_parameter('stub_soc_pct', 100.0)
        self.declare_parameter('stub_voltage_v', 13.2)
        self.declare_parameter('stub_current_a', 0.0)

        self.wh_remaining = self.total_wh
        self.ble_warn_count = 0

        self.period_s = 1.0 / float(em.get('loop_rate_hz', 1.0))
        self.status_pub = self.create_publisher(
            EnergyStatus, '/energy/status', CMD_QOS
        )
        self.timer = self.create_timer(self.period_s, self._publish_status)

    def _publish_status(self) -> None:
        ble_ok = bool(self.get_parameter('bms_ble_connected').value)
        if not ble_ok:
            # TBD-23: loss of BLE only logs warning; never stop the raft.
            self.ble_warn_count += 1
            self.get_logger().warning(
                'BMS Bluetooth LE disconnected; continuing on primary shunt',
                throttle_duration_sec=10.0,
            )

        volt = float(self.get_parameter('stub_voltage_v').value)
        curr = float(self.get_parameter('stub_current_a').value)
        if curr > 0.0:
            self.wh_remaining = integrate_shunt_wh(
                self.wh_remaining, volt, curr, self.period_s, self.total_wh
            )
            soc = 100.0 * self.wh_remaining / self.total_wh
        else:
            soc = float(self.get_parameter('stub_soc_pct').value)
            self.wh_remaining = self.total_wh * (soc / 100.0)

        msg = EnergyStatus()
        msg.stamp = self.now_msg()
        msg.soc_pct = soc
        msg.voltage_v = volt
        msg.current_a = curr
        msg.wh_remaining = self.wh_remaining
        msg.level = classify_energy_level(soc, volt, self.cfg)
        msg.bms_ble_connected = ble_ok
        self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(EnergyNode, args)
