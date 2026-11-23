"""Complete S0-S12 Mission FSM engine with transition table, hard priority, and 3-tier faults.

Implements CdCT Final 2.0 (SPEC_PHU_LUC_A.md §5, PROJECT_RULES.md §3 rule 5, A.8/A.13):
  - All 13 states S0..S12 (never collapsed to S0..S10).
  - Hard priority order: S9 > S8 > S12 > S7 > S11 > S10 > S1.
  - 3 fault tiers:
      Tier 1: self-recovering minor fault (e.g. BMS BLE loss, energy WARN).
      Tier 2: moderate fault -> DDKC corridor emergency berth (S12).
      Tier 3: critical fault -> immediate safe hold/stop (S8/S9) + HOLD-ALL <= 2 s.
  - Strict transition table validation (zero out-of-table transitions).
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from nenuphar_msgs.msg import EnergyStatus, MissionState

# State aliases for readability.
S0 = MissionState.STATE_S0    # INIT_PRECHECK (USB 3 & health gate)
S1 = MissionState.STATE_S1    # MISSION_PATROL
S2 = MissionState.STATE_S2    # OBSTACLE_AVOID (Config A 400P <= 200 ms)
S3 = MissionState.STATE_S3    # BRIDGE_APPROACH (<= 2 km/h, S3 timeout)
S4 = MissionState.STATE_S4    # BRIDGE_CLEARANCE_MEASURE (Config B 800P, +/- 5 cm)
S5 = MissionState.STATE_S5    # BRIDGE_TRANSIT (requires bridge_permit, VIO EKF)
S6 = MissionState.STATE_S6    # BRIDGE_EXIT_REACQUIRE (GNSS reacquire, S6 timeout)
S7 = MissionState.STATE_S7    # FORMATION_HOLD_RESYNC
S8 = MissionState.STATE_S8    # SAFE_HOLD_TIER3 (HOLD-ALL <= 2 s)
S9 = MissionState.STATE_S9    # EMERGENCY_STOP (HOLD-ALL <= 2 s)
S10 = MissionState.STATE_S10  # RTB_RETURN_TO_BASE
S11 = MissionState.STATE_S11  # DOCKING_STANDBY
S12 = MissionState.STATE_S12  # DDKC_EMERGENCY_BERTH (Tier 2)

ALL_STATES: Tuple[int, ...] = (
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12
)

# Hard priority order from PROJECT_RULES.md §3 rule 5.
PRIORITY_ORDER: Tuple[int, ...] = (S9, S8, S12, S7, S11, S10, S1)

# Canonical A.8 valid transition table (src, dst).
# Every single edge here must be triggered >= 1 time in SITL per A.13.
ALLOWED_TRANSITIONS: FrozenSet[Tuple[int, int]] = frozenset({
    # Nominal mission & obstacle avoidance
    (S0, S1),
    (S1, S2),
    (S2, S1),
    # Bridge sequence S1 -> S3 -> S4 -> S5 -> S6 -> S1
    (S1, S3),
    (S3, S4),
    (S4, S5),
    (S5, S6),
    (S6, S1),
    # Formation hold/resync S1 <-> S7
    (S1, S7),
    (S7, S1),
    # RTB & Docking S1 -> S10 -> S11 -> S0
    (S1, S10),
    (S10, S11),
    (S11, S0),
    # Tier 2 moderate faults -> DDKC (S12) & recovery to S10/S0
    (S1, S12),
    (S2, S12),
    (S3, S12),
    (S4, S12),
    (S6, S12),
    (S12, S10),
    (S12, S0),
    # Tier 3 critical faults -> Safe Hold (S8) & Emergency Stop (S9)
    (S0, S8),
    (S1, S8),
    (S5, S8),
    (S7, S8),
    (S10, S8),
    (S12, S8),
    (S8, S12),
    (S8, S0),
    # Highest-priority Emergency Stop (S9) from operational/hold states & manual reset to S0
    (S0, S9),
    (S1, S9),
    (S2, S9),
    (S3, S9),
    (S4, S9),
    (S5, S9),
    (S6, S9),
    (S7, S9),
    (S8, S9),
    (S10, S9),
    (S11, S9),
    (S12, S9),
    (S9, S0),
})


@dataclass
class FsmInputs:
    """Consolidated sensor, health, energy, bridge, and fault inputs for one FSM step."""

    now_s: float = 0.0
    usb3_camera_ok: bool = True
    all_nodes_alive: bool = True
    start_mission: bool = False
    obstacle_detected: bool = False
    bridge_detected: bool = False
    at_bridge_measure_point: bool = False
    bridge_clearance_ready: bool = False
    bridge_pass_permitted: bool = False
    bridge_exited: bool = False
    gnss_reacquired: bool = False
    formation_hold_req: bool = False
    rtb_requested: bool = False
    docked_at_base: bool = False
    corridor_deviation_alert: bool = False
    energy_level: int = EnergyStatus.LEVEL_OK
    bms_ble_connected: bool = True
    tier3_critical_fault: bool = False
    emergency_stop_cmd: bool = False
    reset_to_s0: bool = False


@dataclass
class FsmConfig:
    """Tunable timeouts tracked under TBD-20."""

    s3_timeout_s: float = 30.0
    s6_timeout_s: float = 20.0
    hold_all_deadline_s: float = 2.0


@dataclass
class FsmOutput:
    """Outputs of one MissionFsmEngine step."""

    state: int
    fault_tier: int
    bridge_permit: bool
    hold_all_req: bool
    hold_all_latency_s: float
    ekf_source_set: int  # 1 = GNSS, 2 = VIO ODOMETRY
    bridge_zone_active: bool
    max_speed_mps: float
    hold_position: bool


def resolve_fsm_state(
    current_state: int,
    usb3_camera_ok: bool,
    requests: Tuple[int, ...] = (),
) -> int:
    """Resolve state honoring S0 USB-3 gate and hard priority S9>S8>S12>S7>S11>S10>S1."""
    if not usb3_camera_ok and current_state == S0:
        for state in (S9, S8, S12):
            if state in requests:
                return state
        return S0

    for state in PRIORITY_ORDER:
        if state in requests:
            return state
    return current_state


class MissionFsmEngine:
    """Deterministic S0-S12 mission FSM enforcing A.8 transition table and A.13 tracking."""

    def __init__(self, cfg: Optional[FsmConfig] = None) -> None:
        self.cfg = cfg or FsmConfig()
        self.state: int = S0
        self.state_enter_s: float = 0.0
        self.fault_tier: int = 0
        self.bridge_permit: bool = False
        self.hold_all_req: bool = False
        self.hold_all_latency_s: float = 0.0
        self.ekf_source_set: int = 1

        self.observed_transitions: Set[Tuple[int, int]] = set()
        self.transition_counts: Dict[Tuple[int, int], int] = {}
        self.transition_history: List[Tuple[int, int, str]] = []
        self.invalid_transitions: List[Tuple[int, int, str]] = []

    def _transition_to(self, target: int, now_s: float, reason: str) -> bool:
        if target == self.state:
            return False
        edge = (self.state, target)
        if edge not in ALLOWED_TRANSITIONS:
            self.invalid_transitions.append((self.state, target, reason))
            return False
        self.observed_transitions.add(edge)
        self.transition_counts[edge] = self.transition_counts.get(edge, 0) + 1
        self.transition_history.append((self.state, target, reason))
        self.state = target
        self.state_enter_s = now_s
        return True

    def step(self, inp: FsmInputs) -> FsmOutput:
        """Advance FSM by one cycle and return control/safety flags."""
        elapsed_in_state = max(0.0, inp.now_s - self.state_enter_s)

        # Evaluate 3-tier fault classification.
        tier3_stop = inp.emergency_stop_cmd or (inp.energy_level == EnergyStatus.LEVEL_STOP)
        tier3_hold = inp.tier3_critical_fault or (not inp.all_nodes_alive and self.state != S0)
        tier2_ddkc = (
            inp.corridor_deviation_alert
            or (self.state == S3 and elapsed_in_state >= self.cfg.s3_timeout_s)
            or (self.state == S6 and elapsed_in_state >= self.cfg.s6_timeout_s)
            or (
                self.state == S4
                and inp.bridge_clearance_ready
                and not inp.bridge_pass_permitted
            )
        )
        tier1_minor = (not inp.bms_ble_connected) or (
            inp.energy_level == EnergyStatus.LEVEL_WARN
        )

        # Priority 1: S9 Emergency Stop (highest priority across all states).
        if tier3_stop:
            self.fault_tier = 3
            self.bridge_permit = False
            self.hold_all_req = True
            self.hold_all_latency_s = 0.0
            self._transition_to(S9, inp.now_s, 'tier3_emergency_stop')
        # Priority 2: S9 manual reset to S0 once fault cleared.
        elif self.state == S9:
            if inp.reset_to_s0:
                self.fault_tier = 0
                self.hold_all_req = False
                self._transition_to(S0, inp.now_s, 'reset_from_s9')
        # Priority 3: S8 Critical Safe Hold (Tier 3, HOLD-ALL <= 2 s).
        elif tier3_hold and self.state in (S0, S1, S5, S7, S10, S12):
            self.fault_tier = 3
            self.bridge_permit = False
            self.hold_all_req = True
            self.hold_all_latency_s = 0.0
            self._transition_to(S8, inp.now_s, 'tier3_safe_hold')
        elif self.state == S8:
            if inp.reset_to_s0:
                self.fault_tier = 0
                self.hold_all_req = False
                self._transition_to(S0, inp.now_s, 'reset_from_s8')
            elif not tier3_hold and inp.corridor_deviation_alert:
                self.fault_tier = 2
                self.hold_all_req = False
                self._transition_to(S12, inp.now_s, 'deescalate_s8_to_s12')
        # Priority 4: S12 DDKC (Tier 2 moderate fault).
        elif tier2_ddkc and self.state in (S1, S2, S3, S4, S6):
            self.fault_tier = 2
            self.bridge_permit = False
            self.hold_all_req = False
            self._transition_to(S12, inp.now_s, 'tier2_ddkc')
        elif self.state == S12:
            if inp.reset_to_s0:
                self.fault_tier = 0
                self._transition_to(S0, inp.now_s, 'reset_from_s12')
            elif inp.rtb_requested or inp.energy_level == EnergyStatus.LEVEL_RTB:
                self.fault_tier = 1 if tier1_minor else 0
                self._transition_to(S10, inp.now_s, 's12_to_rtb')
        else:
            # Nominal & lower-priority transitions (S7 > S11 > S10 > S1).
            self.hold_all_req = False
            self.fault_tier = 1 if tier1_minor else 0

            if self.state == S0:
                # Reject leaving S0 if camera is not on USB 3 or nodes not alive.
                if inp.start_mission and inp.usb3_camera_ok and inp.all_nodes_alive:
                    self._transition_to(S1, inp.now_s, 'precheck_ok_start')
            elif self.state == S1:
                if inp.formation_hold_req:
                    self._transition_to(S7, inp.now_s, 'formation_hold')
                elif inp.rtb_requested or inp.energy_level == EnergyStatus.LEVEL_RTB:
                    self._transition_to(S10, inp.now_s, 'energy_or_cmd_rtb')
                elif inp.obstacle_detected:
                    self._transition_to(S2, inp.now_s, 'obstacle_avoid')
                elif inp.bridge_detected:
                    self._transition_to(S3, inp.now_s, 'bridge_approach')
            elif self.state == S2:
                if not inp.obstacle_detected:
                    self._transition_to(S1, inp.now_s, 'obstacle_cleared')
            elif self.state == S3:
                if inp.at_bridge_measure_point:
                    self._transition_to(S4, inp.now_s, 'enter_bridge_clearance')
            elif self.state == S4:
                if inp.bridge_clearance_ready and inp.bridge_pass_permitted:
                    self.bridge_permit = True
                    self._transition_to(S5, inp.now_s, 'bridge_permit_granted')
            elif self.state == S5:
                if inp.bridge_exited:
                    self.bridge_permit = False
                    self._transition_to(S6, inp.now_s, 'bridge_exited')
            elif self.state == S6:
                if inp.gnss_reacquired:
                    self._transition_to(S1, inp.now_s, 'gnss_reacquired')
            elif self.state == S7:
                if not inp.formation_hold_req:
                    self._transition_to(S1, inp.now_s, 'formation_resumed')
            elif self.state == S10:
                if inp.docked_at_base:
                    self._transition_to(S11, inp.now_s, 'arrived_at_base')
            elif self.state == S11:
                if inp.reset_to_s0:
                    self._transition_to(S0, inp.now_s, 'docking_complete_standby')

        # Compute EKF source, bridge zone, and speed limits per Final 2.0.
        bridge_zone_active = self.state in (S4, S5)
        self.ekf_source_set = 2 if self.state == S5 else 1

        if self.state in (S0, S7, S8, S9, S11, S12):
            max_speed_mps = 0.0
            hold_position = True
        elif self.state in (S3, S4):
            # SPEC A.11 / A.4: Config B bridge measurement <= 2 km/h (0.555 m/s).
            max_speed_mps = 2.0 / 3.6
            hold_position = False
        else:
            max_speed_mps = 1.5
            hold_position = False

        return FsmOutput(
            state=self.state,
            fault_tier=self.fault_tier,
            bridge_permit=self.bridge_permit,
            hold_all_req=self.hold_all_req,
            hold_all_latency_s=self.hold_all_latency_s,
            ekf_source_set=self.ekf_source_set,
            bridge_zone_active=bridge_zone_active,
            max_speed_mps=max_speed_mps,
            hold_position=hold_position,
        )

    def missing_transitions(self) -> FrozenSet[Tuple[int, int]]:
        """Return valid A.8 transitions not yet triggered."""
        return ALLOWED_TRANSITIONS - self.observed_transitions
