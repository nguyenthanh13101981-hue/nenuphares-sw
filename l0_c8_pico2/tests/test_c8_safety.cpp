#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <queue>

#include "core/lora_protocol.hpp"
#include "core/safety_fsm.hpp"
#include "hal/c8_hal.hpp"

namespace {

using nenuphar::c8::C8Hal;
using nenuphar::c8::C8State;
using nenuphar::c8::LoraCmd;
using nenuphar::c8::LoraPacket;
using nenuphar::c8::LoraVerifier;
using nenuphar::c8::SafetyFsm;
using nenuphar::c8::SafetyInput;
using nenuphar::c8::SafetyOutput;
using nenuphar::c8::encode_lora_packet;
using nenuphar::c8::kBroadcastGroupId;
using nenuphar::c8::kLoraPacketSize;
using nenuphar::c8::kReasonLoraStop;
using nenuphar::c8::kReasonManualEstop;
using nenuphar::c8::kReasonOverload;
using nenuphar::c8::kReasonTilt;
using nenuphar::c8::step_hal_cycle;

constexpr uint16_t kGroupId = 0x0101;
constexpr uint8_t kSharedKey[16] = {
    0x4E, 0x45, 0x4E, 0x55, 0x50, 0x48, 0x41, 0x52,
    0x42, 0x4F, 0x4E, 0x4E, 0x45, 0x56, 0x41, 0x4C,
};

class MockC8Hal : public C8Hal {
 public:
  uint32_t now_ms{0};
  bool manual_estop{false};
  bool manual_reset{false};
  float tilt_deg{0.0f};
  float load_kg{0.0f};
  bool keepalive_pulse{false};
  bool bridge_zone_refresh{false};
  std::queue<std::array<uint8_t, kLoraPacketSize>> lora_rx;

  bool relay_closed{false};
  bool hold_req_output{false};
  uint32_t watchdog_kicks{0};

  uint32_t millis() const override { return now_ms; }
  bool read_manual_estop() const override { return manual_estop; }
  bool read_manual_reset_button() const override { return manual_reset; }
  float read_tilt_deg() const override { return tilt_deg; }
  float read_total_load_kg() const override { return load_kg; }

  bool poll_keepalive_pulse() override {
    const bool p = keepalive_pulse;
    keepalive_pulse = false;
    return p;
  }

  bool poll_bridge_zone_refresh() override {
    const bool r = bridge_zone_refresh;
    bridge_zone_refresh = false;
    return r;
  }

  bool poll_lora_packet(uint8_t out_pkt[12]) override {
    if (lora_rx.empty()) {
      return false;
    }
    const auto pkt = lora_rx.front();
    lora_rx.pop();
    for (size_t i = 0; i < kLoraPacketSize; ++i) {
      out_pkt[i] = pkt[i];
    }
    return true;
  }

  void set_propulsion_relay(bool closed) override { relay_closed = closed; }
  void set_hold_req_output(bool active) override { hold_req_output = active; }
  void kick_external_watchdog() override { ++watchdog_kicks; }
};

TEST(C8SafetyTest, StartsSafeOffAndRequiresKeepaliveToArm) {
  SafetyFsm fsm;
  SafetyInput in;
  in.now_ms = 100;
  in.arm_request = true;

  // No keepalive yet -> stay SAFE_OFF with relay open.
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_FALSE(out.relay_closed);

  // Valid keepalive arrives -> transition to ARMED.
  in.now_ms = 200;
  in.keepalive_pulse = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);
}

TEST(C8SafetyTest, KeepaliveLossThreeSecondsTriggersHoldReqAndRecovers) {
  SafetyFsm fsm;
  SafetyInput in;
  in.now_ms = 1000;
  in.arm_request = true;
  in.keepalive_pulse = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Under 3 s -> still ARMED.
  in.keepalive_pulse = false;
  in.now_ms = 3990;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);

  // Exactly 3000 ms elapsed -> HOLD_REQ (relay stays closed, HOLD_REQ asserted).
  in.now_ms = 4000;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kHoldReq);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_TRUE(out.hold_req_output);

  // Keepalive resumes -> recovers to ARMED.
  in.now_ms = 4100;
  in.keepalive_pulse = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);
}

TEST(C8SafetyTest, Lora12ByteHmacVerificationAndAntiReplay) {
  LoraVerifier verifier(kGroupId, kSharedKey, sizeof(kSharedKey));
  uint8_t pkt[kLoraPacketSize]{};
  LoraPacket parsed;

  // Valid RUN packet for local group.
  encode_lora_packet(LoraCmd::kRun, kGroupId, 1, kSharedKey, sizeof(kSharedKey), pkt);
  ASSERT_TRUE(verifier.verify_and_parse(pkt, parsed));
  EXPECT_EQ(parsed.cmd, LoraCmd::kRun);
  EXPECT_EQ(parsed.group_id, kGroupId);
  EXPECT_EQ(parsed.counter, 1U);

  // Anti-replay: counter <= last_valid_counter must be rejected.
  EXPECT_FALSE(verifier.verify_and_parse(pkt, parsed));

  // Tampered HMAC byte must be rejected and must not advance counter.
  uint8_t bad_hmac[kLoraPacketSize]{};
  encode_lora_packet(LoraCmd::kStop, kGroupId, 2, kSharedKey, sizeof(kSharedKey), bad_hmac);
  bad_hmac[11] ^= 0x01U;
  EXPECT_FALSE(verifier.verify_and_parse(bad_hmac, parsed));
  EXPECT_EQ(verifier.last_valid_counter(), 1U);

  // Wrong group ID rejected.
  uint8_t wrong_grp[kLoraPacketSize]{};
  encode_lora_packet(LoraCmd::kStop, 0x0202, 2, kSharedKey, sizeof(kSharedKey), wrong_grp);
  EXPECT_FALSE(verifier.verify_and_parse(wrong_grp, parsed));

  // Broadcast group ID with counter=2 accepted after earlier rejections.
  uint8_t bcast[kLoraPacketSize]{};
  encode_lora_packet(
      LoraCmd::kHold, kBroadcastGroupId, 2, kSharedKey, sizeof(kSharedKey), bcast);
  ASSERT_TRUE(verifier.verify_and_parse(bcast, parsed));
  EXPECT_EQ(parsed.cmd, LoraCmd::kHold);
  EXPECT_EQ(parsed.counter, 2U);
}

TEST(C8SafetyTest, BridgeZoneSuppressesAutoFaultUpTo60sOnlyNeverEstopOrLoraStop) {
  SafetyFsm fsm;
  SafetyInput in;
  in.now_ms = 0;
  in.arm_request = true;
  in.keepalive_pulse = true;
  in.bridge_zone_refresh = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Within 60 s BRIDGE_ZONE: overload (>= 10 kg) and tilt are suppressed.
  in.now_ms = 2000;
  in.keepalive_pulse = true;
  in.bridge_zone_refresh = false;
  in.load_kg = 12.5f;
  in.tilt_deg = 30.0f;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.bridge_zone_active);
  EXPECT_TRUE(out.relay_closed);

  // Beyond 60 s without refresh: auto-cutoff engages immediately -> STOPPED.
  in.now_ms = 60001;
  in.keepalive_pulse = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.bridge_zone_active);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_EQ(out.stop_reason_flags, kReasonOverload | kReasonTilt);

  // Fresh FSM: active BRIDGE_ZONE must NEVER block LoRa STOP.
  SafetyFsm fsm_lora;
  SafetyInput in2;
  in2.now_ms = 0;
  in2.arm_request = true;
  in2.keepalive_pulse = true;
  in2.bridge_zone_refresh = true;
  ASSERT_EQ(fsm_lora.step(in2).state, C8State::kArmed);

  in2.now_ms = 1000;
  in2.bridge_zone_refresh = false;
  in2.has_lora_cmd = true;
  in2.lora_cmd = LoraCmd::kStop;
  out = fsm_lora.step(in2);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonLoraStop) != 0);
}

TEST(C8SafetyTest, StoppedIsLatchedUntilManualResetAtRaft) {
  SafetyFsm fsm;
  SafetyInput in;
  in.now_ms = 0;
  in.arm_request = true;
  in.keepalive_pulse = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Trigger manual E-stop.
  in.now_ms = 100;
  in.manual_estop = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kStopped);

  // Releasing E-stop + LoRa RUN + keepalive cannot unlatch STOPPED.
  in.now_ms = 200;
  in.manual_estop = false;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.keepalive_pulse = true;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);

  // Manual reset at raft brings it back to SAFE_OFF (relay remains open).
  in.now_ms = 300;
  in.has_lora_cmd = false;
  in.manual_reset_at_raft = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_FALSE(out.relay_closed);
}

TEST(C8SafetyTest, ThirtyConsecutiveEstopTrialsHundredPercentCutoffViaHal) {
  MockC8Hal hal;
  SafetyFsm fsm;
  LoraVerifier lora(kGroupId, kSharedKey, sizeof(kSharedKey));
  uint32_t lora_counter = 1;
  uint32_t estop_success_count = 0;

  for (int trial = 0; trial < 30; ++trial) {
    // 1. Ensure SAFE_OFF -> ARMED before each E-stop trial.
    hal.manual_estop = false;
    hal.manual_reset = false;
    hal.tilt_deg = 0.0f;
    hal.load_kg = 2.0f;
    hal.keepalive_pulse = true;
    hal.bridge_zone_refresh = (trial % 3 == 0);  // Exercise BRIDGE_ZONE active too.
    hal.now_ms += 100;

    SafetyOutput out = step_hal_cycle(hal, fsm, lora, true);
    ASSERT_EQ(out.state, C8State::kArmed) << "Trial " << trial << " failed to arm";
    ASSERT_TRUE(hal.relay_closed);

    // 2. Alternate 15 manual E-stop button presses and 15 authenticated LoRa STOP frames.
    hal.now_ms += 20;
    if (trial % 2 == 0) {
      hal.manual_estop = true;
    } else {
      std::array<uint8_t, kLoraPacketSize> pkt{};
      encode_lora_packet(
          LoraCmd::kStop, kGroupId, lora_counter++, kSharedKey, sizeof(kSharedKey), pkt.data());
      hal.lora_rx.push(pkt);
    }

    out = step_hal_cycle(hal, fsm, lora, true);
    if (out.state == C8State::kStopped && !hal.relay_closed) {
      ++estop_success_count;
    }

    // 3. Release E-stop and perform manual reset at raft for next trial.
    hal.manual_estop = false;
    hal.now_ms += 20;
    hal.manual_reset = true;
    out = step_hal_cycle(hal, fsm, lora, false);
    ASSERT_EQ(out.state, C8State::kSafeOff);
    ASSERT_FALSE(hal.relay_closed);
  }

  EXPECT_EQ(estop_success_count, 30U);
  EXPECT_EQ(hal.watchdog_kicks, 90U);
}

}  // namespace
