#include "core/safety_fsm.hpp"

#include <cmath>

namespace nenuphar::c8 {
namespace {

bool is_tilt_valid(const TiltSample& s, uint32_t now_ms, uint32_t stale_ms) {
  if (!s.valid || !std::isfinite(s.roll_deg) || !std::isfinite(s.pitch_deg)) {
    return false;
  }
  if (std::fabs(s.roll_deg) > 180.0f || std::fabs(s.pitch_deg) > 180.0f) {
    return false;
  }
  return static_cast<uint32_t>(now_ms - s.timestamp_ms) <= stale_ms;
}

bool is_load_valid(const LoadSample& s, uint32_t now_ms, uint32_t stale_ms) {
  if (!s.valid || !std::isfinite(s.total_load_kg)) {
    return false;
  }
  if (s.total_load_kg < -5.0f || s.total_load_kg > 250.0f) {
    return false;
  }
  return static_cast<uint32_t>(now_ms - s.timestamp_ms) <= stale_ms;
}

}  // namespace

SafetyFsm::SafetyFsm(const SafetyConfig& cfg) : cfg_(cfg) {}

SafetyOutput SafetyFsm::step(const SafetyInput& in) {
  const bool reset_rising = in.manual_reset_at_raft && !prev_reset_at_raft_;
  const bool start_rising = in.manual_start_at_raft && !prev_start_at_raft_;

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

  if (lora_run) {
    has_lora_run_ = true;
    last_lora_run_ms_ = in.now_ms;
    hold_from_lora_ = false;
  } else if (lora_hold) {
    hold_from_lora_ = true;
  }

  const bool lora_run_fresh =
      has_lora_run_ &&
      static_cast<uint32_t>(in.now_ms - last_lora_run_ms_) <= cfg_.lora_run_freshness_ms;

  const bool tilt_ok = is_tilt_valid(in.tilt, in.now_ms, cfg_.sensor_stale_timeout_ms);
  const bool load_ok = is_load_valid(in.load, in.now_ms, cfg_.sensor_stale_timeout_ms);
  const bool sensor_fault = !tilt_ok || !load_ok;

  if (load_ok && (!has_tare_ || (state_ == C8State::kSafeOff && in.tare_request))) {
    tare_load_kg_ = in.load.total_load_kg;
    has_tare_ = true;
  }

  bool dyn_tilt_fault = false;
  bool stat_tilt_fault = false;
  if (tilt_ok) {
    const float peak_tilt = std::fmax(std::fabs(in.tilt.roll_deg), std::fabs(in.tilt.pitch_deg));
    if (peak_tilt > cfg_.dynamic_tilt_deg) {
      if (!dyn_tilt_active_) {
        dyn_tilt_active_ = true;
        dyn_tilt_start_ms_ = in.now_ms;
      }
      if (static_cast<uint32_t>(in.now_ms - dyn_tilt_start_ms_) >= cfg_.dynamic_tilt_window_ms) {
        dyn_tilt_fault = true;
      }
    } else {
      dyn_tilt_active_ = false;
    }

    if (peak_tilt > cfg_.static_tilt_deg) {
      if (!stat_tilt_active_) {
        stat_tilt_active_ = true;
        stat_tilt_start_ms_ = in.now_ms;
      }
      if (static_cast<uint32_t>(in.now_ms - stat_tilt_start_ms_) > cfg_.static_tilt_window_ms) {
        stat_tilt_fault = true;
      }
    } else {
      stat_tilt_active_ = false;
    }
  } else {
    dyn_tilt_active_ = false;
    stat_tilt_active_ = false;
  }

  bool load_delta_fault = false;
  if (load_ok && has_tare_) {
    const float load_delta = std::fabs(in.load.total_load_kg - tare_load_kg_);
    if (load_delta >= cfg_.max_load_delta_kg) {
      if (!load_delta_active_) {
        load_delta_active_ = true;
        load_delta_start_ms_ = in.now_ms;
      }
      if (static_cast<uint32_t>(in.now_ms - load_delta_start_ms_) >= cfg_.load_delta_window_ms) {
        load_delta_fault = true;
      }
    } else {
      load_delta_active_ = false;
    }
  } else {
    load_delta_active_ = false;
  }

  // Hard stops: NEVER suppressed by BRIDGE_ZONE.
  uint8_t fault_flags = kReasonNone;
  if (in.manual_estop) {
    fault_flags |= kReasonManualEstop;
  }
  if (lora_stop) {
    fault_flags |= kReasonLoraStop;
  }
  if (in.stop_from_4g) {
    fault_flags |= kReason4gStop;
  }
  if (in.stop_from_watchdog) {
    fault_flags |= kReasonWatchdogStop;
  }

  // Auto cutoffs: suppressed ONLY when BRIDGE_ZONE is active and refreshed <= 60 s.
  if (!bridge_zone_active) {
    if (load_delta_fault) {
      fault_flags |= kReasonLoadDelta;
    }
    if (dyn_tilt_fault) {
      fault_flags |= kReasonDynamicTilt;
    }
    if (stat_tilt_fault) {
      fault_flags |= kReasonStaticTilt;
    }
    if (sensor_fault && (state_ == C8State::kArmed || state_ == C8State::kHoldReq)) {
      fault_flags |= kReasonSensorFault;
    }
  }

  if (state_ == C8State::kStopped) {
    if (fault_flags != kReasonNone) {
      stop_reason_flags_ |= fault_flags;
    } else if (!sensor_fault && reset_rising) {
      // EN ISO 13850: only rising-edge manual reset at raft exits latched STOPPED -> SAFE_OFF.
      // Requires a fresh start button press + fresh LoRa RUN before contactor can re-close.
      state_ = C8State::kSafeOff;
      hold_from_lora_ = false;
      has_lora_run_ = false;
      stop_reason_flags_ = kReasonNone;
    }
  } else if (fault_flags != kReasonNone) {
    state_ = C8State::kStopped;
    hold_from_lora_ = false;
    has_lora_run_ = false;
    stop_reason_flags_ = fault_flags;
  } else {
    switch (state_) {
      case C8State::kSafeOff:
        // CdCT A.10: C4 cannot arm contactor. Must see start rising edge + valid sensors +
        // keepalive + recent LoRa RUN + E-stop released.
        if (start_rising && !sensor_fault && keepalive_ok && lora_run_fresh && !hold_from_lora_) {
          state_ = C8State::kArmed;
        }
        break;
      case C8State::kArmed:
        if (in.manual_disarm) {
          hold_from_lora_ = false;
          state_ = C8State::kSafeOff;
        } else if (!keepalive_ok || hold_from_lora_) {
          state_ = C8State::kHoldReq;
        }
        break;
      case C8State::kHoldReq:
        if (in.manual_disarm) {
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

  prev_reset_at_raft_ = in.manual_reset_at_raft;
  prev_start_at_raft_ = in.manual_start_at_raft;

  SafetyOutput out;
  out.state = state_;
  out.relay_closed = (state_ == C8State::kArmed || state_ == C8State::kHoldReq);
  out.hold_req_output = (state_ == C8State::kHoldReq);
  out.bridge_zone_active = bridge_zone_active;
  out.sensor_fault = sensor_fault;
  out.stop_reason_flags = stop_reason_flags_;
  return out;
}

SafetyOutput step_hal_cycle(
    C8Hal& hal,
    SafetyFsm& fsm,
    LoraVerifier& lora,
    bool manual_disarm) {
  SafetyInput in;
  in.now_ms = hal.millis();
  in.manual_estop = hal.read_manual_estop();
  in.manual_reset_at_raft = hal.read_manual_reset_button();
  in.manual_start_at_raft = hal.read_start_button();
  in.manual_disarm = manual_disarm;
  in.stop_from_4g = hal.read_4g_stop();
  in.stop_from_watchdog = hal.read_watchdog_stop();
  in.tilt = hal.read_tilt_sample();
  in.load = hal.read_load_sample();
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
  if (hal.cycle_health_ok()) {
    hal.kick_external_watchdog();
  }
  return out;
}

}  // namespace nenuphar::c8
