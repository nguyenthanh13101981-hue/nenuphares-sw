#include "core/safety_fsm.hpp"

#include <cmath>

namespace nenuphar::c8 {

SafetyFsm::SafetyFsm(const SafetyConfig& cfg) : cfg_(cfg) {}

SafetyOutput SafetyFsm::step(const SafetyInput& in) {
  if (in.keepalive_pulse) {
    has_keepalive_ = true;
    last_keepalive_ms_ = in.now_ms;
  }
  if (in.bridge_zone_refresh) {
    has_bridge_zone_ = true;
    last_bridge_zone_ms_ = in.now_ms;
  }

  const bool keepalive_ok =
      has_keepalive_ &&
      static_cast<uint32_t>(in.now_ms - last_keepalive_ms_) < cfg_.keepalive_timeout_ms;
  const bool bridge_zone_active =
      has_bridge_zone_ &&
      static_cast<uint32_t>(in.now_ms - last_bridge_zone_ms_) <= cfg_.bridge_zone_timeout_ms;

  const bool lora_stop = in.has_lora_cmd && in.lora_cmd == LoraCmd::kStop;
  const bool lora_hold = in.has_lora_cmd && in.lora_cmd == LoraCmd::kHold;
  const bool lora_run = in.has_lora_cmd && in.lora_cmd == LoraCmd::kRun;

  const bool overload = in.load_kg >= cfg_.max_load_kg;
  const bool over_tilt = std::fabs(in.tilt_deg) >= cfg_.max_tilt_deg;

  // BRIDGE_ZONE only suppresses auto-cutoff from tilt/load; never manual E-stop or LoRa STOP.
  uint8_t fault_flags = kReasonNone;
  if (in.manual_estop) {
    fault_flags |= kReasonManualEstop;
  }
  if (lora_stop) {
    fault_flags |= kReasonLoraStop;
  }
  if (!bridge_zone_active && overload) {
    fault_flags |= kReasonOverload;
  }
  if (!bridge_zone_active && over_tilt) {
    fault_flags |= kReasonTilt;
  }

  if (state_ == C8State::kStopped) {
    if (fault_flags != kReasonNone) {
      stop_reason_flags_ |= fault_flags;
    } else if (in.manual_reset_at_raft) {
      // EN ISO 13850: only manual reset at raft exits latched STOPPED -> SAFE_OFF.
      state_ = C8State::kSafeOff;
      hold_from_lora_ = false;
      stop_reason_flags_ = kReasonNone;
    }
  } else if (fault_flags != kReasonNone) {
    state_ = C8State::kStopped;
    hold_from_lora_ = false;
    stop_reason_flags_ = fault_flags;
  } else {
    if (lora_hold) {
      hold_from_lora_ = true;
    } else if (lora_run) {
      hold_from_lora_ = false;
    }

    switch (state_) {
      case C8State::kSafeOff:
        if (in.arm_request && keepalive_ok && !hold_from_lora_) {
          state_ = C8State::kArmed;
        }
        break;
      case C8State::kArmed:
        if (!in.arm_request) {
          hold_from_lora_ = false;
          state_ = C8State::kSafeOff;
        } else if (!keepalive_ok || hold_from_lora_) {
          state_ = C8State::kHoldReq;
        }
        break;
      case C8State::kHoldReq:
        if (!in.arm_request) {
          hold_from_lora_ = false;
          state_ = C8State::kSafeOff;
        } else if (keepalive_ok && !hold_from_lora_) {
          state_ = C8State::kArmed;
        }
        break;
      case C8State::kStopped:
        break;
    }
  }

  SafetyOutput out;
  out.state = state_;
  out.relay_closed = (state_ == C8State::kArmed || state_ == C8State::kHoldReq);
  out.hold_req_output = (state_ == C8State::kHoldReq);
  out.bridge_zone_active = bridge_zone_active;
  out.stop_reason_flags = stop_reason_flags_;
  return out;
}

SafetyOutput step_hal_cycle(
    C8Hal& hal,
    SafetyFsm& fsm,
    LoraVerifier& lora,
    bool arm_request) {
  SafetyInput in;
  in.now_ms = hal.millis();
  in.arm_request = arm_request;
  in.manual_estop = hal.read_manual_estop();
  in.manual_reset_at_raft = hal.read_manual_reset_button();
  in.tilt_deg = hal.read_tilt_deg();
  in.load_kg = hal.read_total_load_kg();
  in.keepalive_pulse = hal.poll_keepalive_pulse();
  in.bridge_zone_refresh = hal.poll_bridge_zone_refresh();

  uint8_t raw_pkt[kLoraPacketSize]{};
  if (hal.poll_lora_packet(raw_pkt)) {
    LoraPacket parsed;
    if (lora.verify_and_parse(raw_pkt, parsed)) {
      in.has_lora_cmd = true;
      in.lora_cmd = parsed.cmd;
    }
  }

  const SafetyOutput out = fsm.step(in);
  hal.set_propulsion_relay(out.relay_closed);
  hal.set_hold_req_output(out.hold_req_output);
  hal.kick_external_watchdog();
  return out;
}

}  // namespace nenuphar::c8
