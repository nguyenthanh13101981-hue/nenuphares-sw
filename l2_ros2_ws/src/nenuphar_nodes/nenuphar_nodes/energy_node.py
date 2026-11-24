"""Energy monitor node (1 Hz, real L1 shunt stream + A.9.3 WARN hysteresis + BLE warn-only)."""

from nenuphar_msgs.msg import EnergyStatus

from nenuphar_nodes.base_node import MonitoredNode, run_node_main
from nenuphar_nodes.config_loader import load_energy_config
from nenuphar_nodes.energy_logic import (
    EnergyClassifier,
    classify_energy_level,
    integrate_shunt_wh,
)
from nenuphar_nodes.qos import CMD_QOS

__all__ = ['EnergyNode', 'EnergyClassifier', 'classify_energy_level', 'integrate_shunt_wh']


class EnergyNode(MonitoredNode):
    """Publishes 1 Hz /energy/status from real /l1/battery_shunt or stub params."""

    def __init__(self, **kwargs) -> None:
        super().__init__('energy', **kwargs)
        self.cfg = load_energy_config()
        em = self.cfg['energy_management']
        self.total_wh = float(em['total_capacity_wh'])
        self.classifier = EnergyClassifier(self.cfg)

        self.declare_parameter('use_stub_inputs', True)
        self.declare_parameter('bms_ble_connected', True)
        self.declare_parameter('stub_soc_pct', 100.0)
        self.declare_parameter('stub_voltage_v', 13.2)
        self.declare_parameter('stub_current_a', 0.0)

        self.wh_remaining = self.total_wh
        self.ble_warn_count = 0
        self._has_live_shunt = False
        self._live_voltage_v = 13.2
        self._live_current_a = 0.0
        self._live_soc_pct = 100.0

        self.period_s = 1.0 / float(em.get('loop_rate_hz', 1.0))
        self.shunt_sub = self.create_subscription(
            EnergyStatus, '/l1/battery_shunt', self._on_live_shunt, CMD_QOS
        )
        self.status_pub = self.create_publisher(
            EnergyStatus, '/energy/status', CMD_QOS
        )
        self.timer = self.create_timer(self.period_s, self._publish_status)

    def _on_live_shunt(self, msg: EnergyStatus) -> None:
        self._has_live_shunt = True
        self._live_voltage_v = float(msg.voltage_v)
        self._live_current_a = float(msg.current_a)
        self._live_soc_pct = float(msg.soc_pct)

    def _publish_status(self) -> None:
        ble_ok = bool(self.get_parameter('bms_ble_connected').value)
        if not ble_ok:
            # TBD-23: loss of BLE only logs warning; never stop the raft.
            self.ble_warn_count += 1
            self.get_logger().warning(
                'BMS Bluetooth LE disconnected; continuing on primary shunt',
                throttle_duration_sec=10.0,
            )

        use_stub = bool(self.get_parameter('use_stub_inputs').value) and not self._has_live_shunt
        if self._has_live_shunt:
            volt = self._live_voltage_v
            curr = self._live_current_a
            if curr > 0.0:
                self.wh_remaining = integrate_shunt_wh(
                    self.wh_remaining, volt, curr, self.period_s, self.total_wh
                )
                soc = 100.0 * self.wh_remaining / self.total_wh
            else:
                soc = self._live_soc_pct
                self.wh_remaining = self.total_wh * (soc / 100.0)
        elif use_stub:
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
        else:
            volt = 0.0
            curr = 0.0
            soc = 0.0

        msg = EnergyStatus()
        msg.stamp = self.now_msg()
        msg.soc_pct = soc
        msg.voltage_v = volt
        msg.current_a = curr
        msg.wh_remaining = self.wh_remaining
        msg.level = self.classifier.classify(soc, volt)
        msg.bms_ble_connected = ble_ok
        self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node_main(EnergyNode, args)
