"""Complete CdCT Final A.8 S0-S12 Mission FSM engine loaded from docs/cdct_spec_a8_a10.yaml.

Enforces exact CdCT Final A.8 state meanings, transitions, hard priority order, and 3-tier faults:
  - S0: INIT_PRECHECK (USB 3 & liveness gate)
  - S1: MISSION_PATROL
  - S2: BRIDGE_APPROACH (>= 10 m before bridge, <= 2 km/h)
  - S3: BRIDGE_CENTERING (+/- 25 cm, timeout 30 s -> S7)
  - S4: BRIDGE_CLEARANCE_MEASURE (Config B 800P, >= raft_height + 0.10 m, DB +/- 0.05 m)
  - S5: BRIDGE_TRANSIT (bridge_permit + BRIDGE_ZONE + VIO EKF source 2)
  - S6: BRIDGE_EXIT (RTK reacquire; timeout > 30 s -> S12 / S7 + HOLD-ALL)
  - S7: HOLD_WAIT (Wait / Hold: S3 timeout, S4 unpermitted, RTK/cam loss, outside Z2, HOLD-ALL, docked)
  - S8: BRIDGE_EMERGENCY_EXIT (Emergency exit from under bridge with propulsion intact)
  - S9: EMERGENCY_STOP (Total propulsion loss / E-stop / critical fault outside bridge + HOLD-ALL <= 2 s)
  - S10: RTB_RETURN_TO_BASE (Return to base; docked -> S7)
  - S11: OBSTACLE_AVOID (g >= 1.35 m, within Z2 +/- 1.25 m, <= 10 s; preempts S10)
  - S12: DDKC_EMERGENCY_BERTH (Tier 2 DDKC: Z1 excursion, SoC <= 10% w/ propulsion, S6 RTK timeout, S11 fail)
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from nenuphar_nodes.config_loader import load_cdct_spec
from nenuphar_nodes.energy_logic import LEVEL_OK, LEVEL_RTB, LEVEL_STOP, LEVEL_WARN

# Exact CdCT Final A.8 state aliases.
S0 = 0    # INIT_PRECHECK
S1 = 1    # MISSION_PATROL
S2 = 2    # BRIDGE_APPROACH
S3 = 3    # BRIDGE_CENTERING
S4 = 4    # BRIDGE_CLEARANCE_MEASURE
S5 = 5    # BRIDGE_TRANSIT
S6 = 6    # BRIDGE_EXIT
S7 = 7    # HOLD_WAIT
S8 = 8    # BRIDGE_EMERGENCY_EXIT
S9 = 9    # EMERGENCY_STOP
S10 = 10  # RTB_RETURN_TO_BASE
S11 = 11  # OBSTACLE_AVOID
S12 = 12  # DDKC_EMERGENCY_BERTH

ALL_STATES: Tuple[int, ...] = (
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12
)

_SPEC = load_cdct_spec()
_A8 = _SPEC['a8_fsm']

PRIORITY_ORDER: Tuple[int, ...] = tuple(int(s) for s in _A8['priority_order'])
ALLOWED_TRANSITIONS: FrozenSet[Tuple[int, int]] = frozenset(
    (int(edge[0]), int(edge[1])) for edge in _A8['allowed_transitions']
)
STATE_NAMES: Dict[int, str] = {
    int(k): str(v['name']) for k, v in _A8['states'].items()
}


@dataclass
class FsmInputs:
    """Consolidated sensor, health, energy, bridge, obstacle, and fault inputs for one FSM step."""

    now_s: float = 0.0
    fault_detected_s: Optional[float] = None
    usb3_camera_ok: bool = True
    all_nodes_alive: bool = True
    start_mission: bool = False
    obstacle_detected: bool = False
    obstacle_gap_m: float = 2.0
    bridge_detected: bool = False
    at_bridge_centering: bool = False
    bridge_centered: bool = False
    bridge_clearance_ready: bool = False
    bridge_pass_permitted: bool = False
    bridge_id: str = ''
    bridge_exited: bool = False
    gnss_reacquired: bool = False
    hold_requested: bool = False
    rtb_requested: bool = False
    docked_at_base: bool = False
    corridor_ddkc_alert: bool = False
    outside_z2_hold: bool = False
    energy_level: int = LEVEL_OK
    bms_ble_connected: bool = True
    propulsion_available: bool = True
    tier3_critical_fault: bool = False
    emergency_stop_cmd: bool = False
    reset_to_s0: bool = False


@dataclass
class FsmConfig:
    """Timeouts sourced from docs/cdct_spec_a8_a10.yaml."""

    s3_centering_timeout_s: float = float(_A8['timeouts']['s3_centering_timeout_s'])
    s6_rtk_timeout_s: float = float(_A8['timeouts']['s6_rtk_timeout_s'])
    s11_avoid_max_s: float = float(_A8['timeouts']['s11_avoid_max_s'])
    hold_all_deadline_s: float = float(_A8['timeouts']['hold_all_deadline_s'])
    bridge_permit_ttl_s: float = float(_A8['timeouts']['bridge_permit_ttl_s'])
    obstacle_min_gap_m: float = float(_SPEC['tr_4_16_corridor']['obstacle_min_gap_m'])


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
    """Resolve target state honoring S0 USB-3 gate and hard priority S9>S8>S12>S7>S11>S10>S1."""
    if not usb3_camera_ok and current_state == S0:
        for state in (S9, S8, S12, S7):
            if state in requests:
                return state
        return S0

    for state in PRIORITY_ORDER:
        if state in requests:
            return state
    for state in requests:
        return state
    return current_state


class MissionFsmEngine:
    """Deterministic S0-S12 mission FSM enforcing CdCT A.8 table and priority."""

    def __init__(self, cfg: Optional[FsmConfig] = None) -> None:
        self.cfg = cfg or FsmConfig()
        self.state: int = S0
        self.state_enter_s: float = 0.0
        self.fault_tier: int = 0
        self.bridge_permit: bool = False
        self.bridge_permit_granted_s: Optional[float] = None
        self.active_bridge_id: str = ''
        self.hold_all_req: bool = False
        self.hold_all_latency_s: float = 0.0
        self.ekf_source_set: int = 1

        self._fault_onset_s: Optional[float] = None
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

    def _collect_requests(self, inp: FsmInputs, elapsed: float) -> Tuple[List[int], str]:
        reqs: List[int] = []
        reason = 'nominal'

        # 1. Tier 3: Total propulsion loss / E-stop -> S9; critical fault under bridge (S5) w/ propulsion -> S8;
        # critical fault in any other state -> S9 (never ignored in any state!).
        tier3_stop = inp.emergency_stop_cmd or (not inp.propulsion_available)
        tier3_crit = inp.tier3_critical_fault or (not inp.all_nodes_alive and self.state != S0)
        if tier3_stop:
            reqs.append(S9)
            reason = 'tier3_emergency_stop'
        elif tier3_crit:
            if self.state == S5 and inp.propulsion_available:
                reqs.append(S8)
                reason = 'tier3_bridge_emergency_exit'
            elif self.state != S8:
                reqs.append(S9)
                reason = 'tier3_critical_stop'

        # 2. Tier 2 -> S12 DDKC (includes SoC <= 10% while propulsion intact per CdCT S1).
        s6_rtk_timeout = self.state == S6 and elapsed >= self.cfg.s6_rtk_timeout_s
        s11_avoid_fail = self.state == S11 and elapsed >= self.cfg.s11_avoid_max_s
        soc_ddkc = inp.energy_level == LEVEL_STOP and inp.propulsion_available
        if inp.corridor_ddkc_alert or s6_rtk_timeout or s11_avoid_fail or soc_ddkc:
            if self.state in (S1, S2, S3, S4, S6, S7, S8, S10, S11):
                reqs.append(S12)
                reason = 'tier2_ddkc'

        # 3. Hold / Wait -> S7.
        s3_timeout = self.state == S3 and elapsed >= self.cfg.s3_centering_timeout_s
        s4_denied = (
            self.state == S4
            and inp.bridge_clearance_ready
            and not inp.bridge_pass_permitted
        )
        s10_docked = self.state == S10 and inp.docked_at_base
        if (
            inp.hold_requested
            or inp.outside_z2_hold
            or s3_timeout
            or s4_denied
            or s10_docked
        ):
            if self.state in (S1, S2, S3, S4, S6, S8, S10, S11, S12):
                reqs.append(S7)
                reason = 'hold_wait'

        # 4. Obstacle Avoidance -> S11 (g >= 1.35 m, preempts S10 & S1 per PRIORITY_ORDER).
        if (
            inp.obstacle_detected
            and inp.obstacle_gap_m >= self.cfg.obstacle_min_gap_m
            and self.state in (S1, S10)
        ):
            reqs.append(S11)
            reason = 'obstacle_avoid_s11'
        elif self.state == S11 and not inp.obstacle_detected:
            if inp.rtb_requested or inp.energy_level == LEVEL_RTB:
                reqs.append(S10)
                reason = 'obstacle_cleared_rtb'
            else:
                reqs.append(S1)
                reason = 'obstacle_cleared'

        # 5. RTB -> S10.
        if (inp.rtb_requested or inp.energy_level == LEVEL_RTB) and self.state in (
            S1,
            S2,
            S7,
            S12,
        ):
            reqs.append(S10)
            reason = 'rtb_requested'

        # 6. Nominal mission, bridge sequence, and resets.
        if self.state == S0:
            if inp.start_mission and inp.usb3_camera_ok and inp.all_nodes_alive:
                reqs.append(S1)
                reason = 'precheck_ok_start'
        elif self.state == S1:
            if inp.bridge_detected:
                reqs.append(S2)
                reason = 'bridge_approach'
        elif self.state == S2:
            if inp.at_bridge_centering:
                reqs.append(S3)
                reason = 'bridge_centering'
        elif self.state == S3:
            if inp.bridge_centered:
                reqs.append(S4)
                reason = 'enter_bridge_clearance'
        elif self.state == S4:
            if inp.bridge_clearance_ready and inp.bridge_pass_permitted:
                reqs.append(S5)
                reason = 'bridge_permit_granted'
        elif self.state == S5:
            if inp.bridge_exited:
                reqs.append(S6)
                reason = 'bridge_exited'
        elif self.state == S6:
            if inp.gnss_reacquired:
                reqs.append(S1)
                reason = 'gnss_reacquired'
        elif self.state == S7:
            if inp.reset_to_s0:
                reqs.append(S0)
                reason = 'reset_from_s7'
            elif (
                inp.bridge_detected
                and inp.at_bridge_centering
                and inp.bridge_centered
                and (not inp.bridge_clearance_ready or inp.bridge_pass_permitted)
            ):
                reqs.append(S4)
                reason = 'retry_bridge_clearance'
            elif (
                not inp.hold_requested
                and not inp.outside_z2_hold
                and not inp.docked_at_base
                and not (inp.bridge_clearance_ready and not inp.bridge_pass_permitted)
            ):
                reqs.append(S1)
                reason = 'resume_from_hold'
        elif self.state == S8:
            if inp.reset_to_s0:
                reqs.append(S0)
                reason = 'reset_from_s8'
            elif inp.bridge_exited:
                reqs.append(S7)
                reason = 'bridge_emergency_exited'
        elif self.state in (S9, S12):
            if inp.reset_to_s0:
                reqs.append(S0)
                reason = 'reset_to_s0'

        return reqs, reason

    def step(self, inp: FsmInputs) -> FsmOutput:
        """Advance FSM by one cycle using centralized priority resolution."""
        elapsed = max(0.0, inp.now_s - self.state_enter_s)
        tier1_minor = (not inp.bms_ble_connected) or (inp.energy_level == LEVEL_WARN)

        # Track fault onset for honest HOLD-ALL latency measurement (B8).
        has_hold_all_fault = (
            inp.emergency_stop_cmd
            or (not inp.propulsion_available)
            or inp.tier3_critical_fault
            or (not inp.all_nodes_alive and self.state != S0)
        )
        if has_hold_all_fault:
            if inp.fault_detected_s is not None:
                self._fault_onset_s = inp.fault_detected_s
            elif self._fault_onset_s is None:
                self._fault_onset_s = inp.now_s
        else:
            self._fault_onset_s = None

        reqs, reason = self._collect_requests(inp, elapsed)
        target = resolve_fsm_state(
            self.state, inp.usb3_camera_ok, tuple(reqs)
        )

        # Prevent S9->S0 reset while an emergency stop / propulsion loss condition is still asserted.
        if self.state == S9 and target == S0 and (inp.emergency_stop_cmd or not inp.propulsion_available):
            target = S9

        prev_state = self.state
        if target != self.state:
            self._transition_to(target, inp.now_s, reason)

        # Manage bridge permit & clear stale clearance whenever exiting S4/S5 (B1).
        if self.state == S5 and prev_state == S4:
            self.bridge_permit = True
            self.bridge_permit_granted_s = inp.now_s
            self.active_bridge_id = inp.bridge_id
        elif self.state != S5:
            self.bridge_permit = False
            self.bridge_permit_granted_s = None

        if self.state in (S8, S9):
            self.fault_tier = 3
            self.hold_all_req = True
            self.hold_all_latency_s = (
                max(0.0, inp.now_s - self._fault_onset_s)
                if self._fault_onset_s is not None
                else 0.0
            )
        elif self.state == S12:
            self.fault_tier = 2
            self.hold_all_req = False
            self.hold_all_latency_s = 0.0
        elif self.state == S7 and (inp.hold_requested or inp.outside_z2_hold):
            self.fault_tier = 1
            self.hold_all_req = False
            self.hold_all_latency_s = 0.0
        else:
            self.fault_tier = 1 if tier1_minor else 0
            self.hold_all_req = False
            self.hold_all_latency_s = 0.0

        bridge_zone_active = self.state in (S4, S5, S8)
        self.ekf_source_set = 2 if self.state in (S5, S8) else 1

        if self.state in (S0, S7, S9, S12):
            max_speed_mps = 0.0
            hold_position = True
        elif self.state in (S2, S3, S4):
            max_speed_mps = 2.0 / 3.6  # <= 2 km/h (0.556 m/s)
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
