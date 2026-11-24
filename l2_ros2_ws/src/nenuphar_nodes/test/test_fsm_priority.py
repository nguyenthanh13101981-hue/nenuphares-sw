"""Unit tests for CdCT Final A.8 Mission FSM states S0-S12 and priority rules."""

from nenuphar_msgs.msg import MissionState
from nenuphar_nodes.fsm_engine import (
    PRIORITY_ORDER,
    S0, S1, S7, S8, S9, S10, S11, S12,
    resolve_fsm_state,
)


def test_all_13_states_s0_to_s12_defined() -> None:
    for idx in range(13):
        assert getattr(MissionState, f'STATE_S{idx}') == idx


def test_usb3_camera_gate_blocks_leaving_s0() -> None:
    next_state = resolve_fsm_state(
        current_state=S0,
        usb3_camera_ok=False,
        requests=(S1,),
    )
    assert next_state == S0

    allowed_state = resolve_fsm_state(
        current_state=S0,
        usb3_camera_ok=True,
        requests=(S1,),
    )
    assert allowed_state == S1


def test_hard_priority_order_s9_gt_s8_gt_s12_gt_s7_gt_s11_gt_s10_gt_s1() -> None:
    expected = (S9, S8, S12, S7, S11, S10, S1)
    assert PRIORITY_ORDER == expected

    for i, high in enumerate(expected):
        for low in expected[i + 1 :]:
            chosen = resolve_fsm_state(
                current_state=S1,
                usb3_camera_ok=True,
                requests=(low, high),
            )
            assert chosen == high
