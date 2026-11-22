#ifndef NENUPHAR_L0_C8_SAFETY_FSM_HPP_
#define NENUPHAR_L0_C8_SAFETY_FSM_HPP_

#include <cstdint>

#include "core/lora_protocol.hpp"
#include "hal/c8_hal.hpp"

namespace nenuphar::c8 {

enum class C8State : uint8_t {
  kSafeOff = 0,
  kArmed = 1,
  kStopped = 2,
  kHoldReq = 3,
};

enum StopReasonFlags : uint8_t {
  kReasonNone = 0,
  kReasonManualEstop = 1U << 0,
  kReasonLoraStop = 1U << 1,
  kReasonOverload = 1U << 2,
  kReasonTilt = 1U << 3,
};

struct SafetyConfig {
  uint32_t keepalive_timeout_ms{3000};
  uint32_t bridge_zone_timeout_ms{60000};
  float max_load_kg{10.0f};
  float max_tilt_deg{25.0f};
};

struct SafetyInput {
  uint32_t now_ms{0};
  bool arm_request{false};
  bool manual_estop{false};
  bool manual_reset_at_raft{false};
  bool keepalive_pulse{false};
  bool bridge_zone_refresh{false};
  bool has_lora_cmd{false};
  LoraCmd lora_cmd{LoraCmd::kRun};
  float tilt_deg{0.0f};
  float load_kg{0.0f};
};

struct SafetyOutput {
  C8State state{C8State::kSafeOff};
  bool relay_closed{false};
  bool hold_req_output{false};
  bool bridge_zone_active{false};
  uint8_t stop_reason_flags{kReasonNone};
};

/** Bare-metal safety finite state machine for board C8 (A.10, EN ISO 13850). */
class SafetyFsm {
 public:
  explicit SafetyFsm(const SafetyConfig& cfg = SafetyConfig{});

  SafetyOutput step(const SafetyInput& in);
  C8State state() const { return state_; }

 private:
  SafetyConfig cfg_;
  C8State state_{C8State::kSafeOff};
  bool hold_from_lora_{false};
  uint8_t stop_reason_flags_{kReasonNone};

  bool has_keepalive_{false};
  uint32_t last_keepalive_ms_{0};

  bool has_bridge_zone_{false};
  uint32_t last_bridge_zone_ms_{0};
};

/** Run one control cycle over C8Hal (>= 100 Hz) and kick the 100 ms HW watchdog. */
SafetyOutput step_hal_cycle(
    C8Hal& hal,
    SafetyFsm& fsm,
    LoraVerifier& lora,
    bool arm_request);

}  // namespace nenuphar::c8

#endif  // NENUPHAR_L0_C8_SAFETY_FSM_HPP_
