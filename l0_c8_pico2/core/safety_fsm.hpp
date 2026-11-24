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
  kReason4gStop = 1U << 2,
  kReasonWatchdogStop = 1U << 3,
  kReasonLoadDelta = 1U << 4,
  kReasonDynamicTilt = 1U << 5,
  kReasonStaticTilt = 1U << 6,
  kReasonSensorFault = 1U << 7,
};

struct SafetyConfig {
  uint32_t keepalive_timeout_ms{3000};
  uint32_t bridge_zone_timeout_ms{60000};
  uint32_t lora_run_freshness_ms{30000};
  uint32_t sensor_stale_timeout_ms{500};

  // TR-8.3 (C4 threshold table):
  //   - Dynamic tilt > 15 deg within < 1 s
  //   - Static roll/pitch > 3 deg for > 1 s
  //   - Load delta >= 10 kg within < 1 s
  float dynamic_tilt_deg{15.0f};
  uint32_t dynamic_tilt_window_ms{200};
  float static_tilt_deg{3.0f};
  uint32_t static_tilt_window_ms{1000};
  float max_load_delta_kg{10.0f};
  uint32_t load_delta_window_ms{200};
};

struct SafetyInput {
  uint32_t now_ms{0};
  bool manual_estop{false};
  bool manual_reset_at_raft{false};
  bool manual_start_at_raft{false};
  bool manual_disarm{false};
  bool tare_request{false};
  bool stop_from_4g{false};
  bool stop_from_watchdog{false};
  bool keepalive_pulse{false};
  bool bridge_zone_refresh{false};
  bool has_lora_cmd{false};
  LoraCmd lora_cmd{LoraCmd::kRun};
  TiltSample tilt{};
  LoadSample load{};
};

struct SafetyOutput {
  C8State state{C8State::kSafeOff};
  bool relay_closed{false};
  bool hold_req_output{false};
  bool bridge_zone_active{false};
  bool sensor_fault{false};
  uint8_t stop_reason_flags{kReasonNone};
};

/** Bare-metal safety finite state machine for board C8 (A.10, TR-8.3, EN ISO 13850). */
class SafetyFsm {
 public:
  explicit SafetyFsm(const SafetyConfig& cfg = SafetyConfig{});

  SafetyOutput step(const SafetyInput& in);
  C8State state() const { return state_; }
  float tare_load_kg() const { return tare_load_kg_; }

 private:
  SafetyConfig cfg_;
  C8State state_{C8State::kSafeOff};
  bool hold_from_lora_{false};
  uint8_t stop_reason_flags_{kReasonNone};

  bool prev_reset_at_raft_{false};
  bool prev_start_at_raft_{false};

  bool has_keepalive_{false};
  uint32_t last_keepalive_ms_{0};

  bool has_bridge_zone_{false};
  uint32_t last_bridge_zone_ms_{0};

  bool has_lora_run_{false};
  uint32_t last_lora_run_ms_{0};

  bool has_tare_{false};
  float tare_load_kg_{0.0f};

  bool dyn_tilt_active_{false};
  uint32_t dyn_tilt_start_ms_{0};

  bool stat_tilt_active_{false};
  uint32_t stat_tilt_start_ms_{0};

  bool load_delta_active_{false};
  uint32_t load_delta_start_ms_{0};
};

/** Run one control cycle over C8Hal (>= 100 Hz) and kick HW watchdog only if healthy. */
SafetyOutput step_hal_cycle(
    C8Hal& hal,
    SafetyFsm& fsm,
    LoraVerifier& lora,
    bool manual_disarm = false);

}  // namespace nenuphar::c8

#endif  // NENUPHAR_L0_C8_SAFETY_FSM_HPP_
