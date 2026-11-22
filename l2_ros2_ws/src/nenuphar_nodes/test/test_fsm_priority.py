"""Unit tests for Mission FSM states S0-S12 and priority rules."""

from nenuphar_msgs.msg import MissionState
from nenuphar_nodes.mission_fsm_node import PRIORITY_ORDER, resolve_fsm_state


def test_all_13_states_s0_to_s12_defined() -> None:
    for idx in range(13):
        assert getattr(MissionState, f'STATE_S{idx}') == idx


def test_usb3_camera_gate_blocks_leaving_s0() -> None:
    next_state = resolve_fsm_state(
        current_state=MissionState.STATE_S0,
        usb3_camera_ok=False,
        requests=(MissionState.STATE_S1,),
    )
    assert next_state == MissionState.STATE_S0

    allowed_state = resolve_fsm_state(
        current_state=MissionState.STATE_S0,
        usb3_camera_ok=True,
        requests=(MissionState.STATE_S1,),
    )
    assert allowed_state == MissionState.STATE_S1


def test_hard_priority_order_s9_gt_s8_gt_s12_gt_s7_gt_s11_gt_s10_gt_s1() -> None:
    expected = (
        MissionState.STATE_S9,
        MissionState.STATE_S8,
        MissionState.STATE_S12,
        MissionState.STATE_S7,
        MissionState.STATE_S11,
        MissionState.STATE_S10,
        MissionState.STATE_S1,
    )
    assert PRIORITY_ORDER == expected

    for i, high in enumerate(expected):
        for low in expected[i + 1 :]:
            chosen = resolve_fsm_state(
                current_state=MissionState.STATE_S1,
                usb3_camera_ok=True,
                requests=(low, high),
            )
            assert chosen == high
