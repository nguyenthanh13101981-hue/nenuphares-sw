"""Pure A.9 / A.9.3 energy Coulomb integration and WARN hysteresis classification."""

from typing import Any, Dict

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_RTB = 2
LEVEL_STOP = 3


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


class EnergyClassifier:
    """Stateful A.9.3 energy classifier implementing hysteresis on WARN (enter 28%, clear 30%)."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        em = cfg['energy_management']
        self.warn_enter_pct = float(em['warn_soc_pct'])
        self.warn_clear_pct = float(em.get('warn_clear_soc_pct', self.warn_enter_pct + 2.0))
        self.rtb_pct = float(em['rtb_soc_pct'])
        self.stop_pct = float(em['stop_soc_pct'])
        self.batt_crt_v = float(em['l1_batt_crt_v_min'])
        self._warn_latched = False

    def classify(self, soc_pct: float, voltage_v: float) -> int:
        if soc_pct <= self.stop_pct or voltage_v <= self.batt_crt_v:
            self._warn_latched = True
            return LEVEL_STOP
        if soc_pct <= self.rtb_pct:
            self._warn_latched = True
            return LEVEL_RTB
        if soc_pct <= self.warn_enter_pct:
            self._warn_latched = True
            return LEVEL_WARN
        if self._warn_latched and soc_pct < self.warn_clear_pct:
            return LEVEL_WARN
        self._warn_latched = False
        return LEVEL_OK


def classify_energy_level(
    soc_pct: float,
    voltage_v: float,
    cfg: Dict[str, Any],
) -> int:
    """Stateless single-sample energy threshold classification."""
    return EnergyClassifier(cfg).classify(soc_pct, voltage_v)
