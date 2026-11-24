#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

#include "core/lora_protocol.hpp"
#include "core/safety_fsm.hpp"
#include "hal/c8_hal.hpp"

namespace {

using nenuphar::c8::C8Hal;
using nenuphar::c8::C8State;
using nenuphar::c8::LoadSample;
using nenuphar::c8::LoraCmd;
using nenuphar::c8::LoraPacket;
using nenuphar::c8::LoraVerifier;
using nenuphar::c8::RamCounterStorage;
using nenuphar::c8::SafetyFsm;
using nenuphar::c8::SafetyInput;
using nenuphar::c8::SafetyOutput;
using nenuphar::c8::TiltSample;
using nenuphar::c8::advance_boot_counter;
using nenuphar::c8::encode_lora_packet;
using nenuphar::c8::encode_lora_packet_from_sender;
using nenuphar::c8::kBroadcastGroupId;
using nenuphar::c8::kLoraPacketSize;
using nenuphar::c8::kReason4gStop;
using nenuphar::c8::kReasonDynamicTilt;
using nenuphar::c8::kReasonLoadDelta;
using nenuphar::c8::kReasonLoraStop;
using nenuphar::c8::kReasonManualEstop;
using nenuphar::c8::kReasonSensorFault;
using nenuphar::c8::kReasonStaticTilt;
using nenuphar::c8::kReasonWatchdogStop;
using nenuphar::c8::step_hal_cycle;

constexpr uint16_t kGroupId = 0x0101;
constexpr uint8_t kSharedKey[16] = {
    0x4E, 0x45, 0x4E, 0x55, 0x50, 0x48, 0x41, 0x52,
    0x42, 0x4F, 0x4E, 0x4E, 0x45, 0x56, 0x41, 0x4C,
};

SafetyInput make_nominal_input(uint32_t now_ms, float base_load_kg = 25.0f) {
  SafetyInput in;
  in.now_ms = now_ms;
  in.tilt = TiltSample{0.0f, 0.0f, now_ms, true};
  in.load = LoadSample{base_load_kg, now_ms, true};
  return in;
}

class MockC8Hal : public C8Hal {
 public:
  uint32_t now_ms{0};
  bool manual_estop{false};
  bool manual_reset{false};
  bool manual_start{false};
  bool stop_4g{false};
  bool stop_watchdog{false};
  TiltSample tilt{0.0f, 0.0f, 0, true};
  LoadSample load{25.0f, 0, true};
  bool keepalive_pulse{false};
  bool bridge_zone_refresh{false};
  bool health_ok{true};
  std::queue<std::array<uint8_t, kLoraPacketSize>> lora_rx;

  bool relay_closed{false};
  bool hold_req_output{false};
  uint32_t watchdog_kicks{0};

  uint32_t millis() const override { return now_ms; }
  bool read_manual_estop() const override { return manual_estop; }
  bool read_manual_reset_button() const override { return manual_reset; }
  bool read_start_button() const override { return manual_start; }
  bool read_4g_stop() const override { return stop_4g; }
  bool read_watchdog_stop() const override { return stop_watchdog; }

  TiltSample read_tilt_sample() const override { return tilt; }
  LoadSample read_load_sample() const override { return load; }

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

  bool cycle_health_ok() const override { return health_ok; }
  void set_propulsion_relay(bool closed) override { relay_closed = closed; }
  void set_hold_req_output(bool active) override { hold_req_output = active; }
  void kick_external_watchdog() override { ++watchdog_kicks; }
};

TEST(C8SafetyTest, StartsSafeOffAndRequiresStartButtonEdgePlusLoraRunAndKeepalive) {
  SafetyFsm fsm;

  // Keepalive alone (C4 cannot arm contactor) -> must remain SAFE_OFF.
  SafetyInput in = make_nominal_input(100);
  in.keepalive_pulse = true;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_FALSE(out.relay_closed);

  // Start button without valid LoRa RUN -> still SAFE_OFF.
  in = make_nominal_input(200);
  in.keepalive_pulse = true;
  in.manual_start_at_raft = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_FALSE(out.relay_closed);

  // Release start button, receive LoRa RUN, then press start button (rising edge) -> ARMED.
  in = make_nominal_input(300);
  in.manual_start_at_raft = false;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);

  in = make_nominal_input(400);
  in.manual_start_at_raft = true;
  in.keepalive_pulse = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);
}

TEST(C8SafetyTest, StuckResetButtonCannotUnlatchStoppedAndDoesNotAutoReclose) {
  SafetyFsm fsm;

  // Arm via proper sequence.
  SafetyInput in = make_nominal_input(100);
  in.keepalive_pulse = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.manual_start_at_raft = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Hold reset button down (stuck) while triggering E-stop -> STOPPED.
  in = make_nominal_input(200);
  in.manual_estop = true;
  in.manual_reset_at_raft = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kStopped);

  // Release E-stop while reset remains stuck high + LoRa RUN + keepalive -> MUST stay STOPPED.
  in = make_nominal_input(300);
  in.manual_estop = false;
  in.manual_reset_at_raft = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.keepalive_pulse = true;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);

  // Release reset, then press reset again (rising edge) -> transitions to SAFE_OFF (relay OPEN).
  in = make_nominal_input(400);
  in.manual_reset_at_raft = false;
  EXPECT_EQ(fsm.step(in).state, C8State::kStopped);

  in = make_nominal_input(500);
  in.manual_reset_at_raft = true;
  in.manual_start_at_raft = true;  // Even if start held, old RUN was cleared on reset.
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_FALSE(out.relay_closed);
}

TEST(C8SafetyTest, Tr83TiltAndTaredLoadWindows) {
  auto arm = [](SafetyFsm& fsm, uint32_t t0, float base_load) {
    SafetyInput in = make_nominal_input(t0, base_load);
    in.keepalive_pulse = true;
    in.has_lora_cmd = true;
    in.lora_cmd = LoraCmd::kRun;
    in.manual_start_at_raft = true;
    ASSERT_EQ(fsm.step(in).state, C8State::kArmed);
  };

  // 1. Transient 5 deg tilt < 1 s must NOT trip static tilt (> 3 deg for > 1 s).
  SafetyFsm fsm1;
  arm(fsm1, 0, 20.0f);
  SafetyInput in = make_nominal_input(500, 20.0f);
  in.tilt.roll_deg = 5.0f;
  EXPECT_EQ(fsm1.step(in).state, C8State::kArmed);

  in = make_nominal_input(1200, 20.0f);
  in.tilt.roll_deg = 5.0f;
  EXPECT_EQ(fsm1.step(in).state, C8State::kArmed);

  // Sustained > 3 deg for > 1000 ms (500 -> 1501 ms) -> STOPPED with kReasonStaticTilt.
  in = make_nominal_input(1501, 20.0f);
  in.tilt.roll_deg = 5.0f;
  SafetyOutput out = fsm1.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonStaticTilt) != 0);

  // 2. Dynamic tilt > 15 deg sustained >= 200 ms (< 1 s) -> STOPPED with kReasonDynamicTilt.
  SafetyFsm fsm2;
  arm(fsm2, 0, 20.0f);
  in = make_nominal_input(100, 20.0f);
  in.tilt.pitch_deg = 16.5f;
  EXPECT_EQ(fsm2.step(in).state, C8State::kArmed);
  in = make_nominal_input(300, 20.0f);
  in.tilt.pitch_deg = 16.5f;
  out = fsm2.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_TRUE((out.stop_reason_flags & kReasonDynamicTilt) != 0);

  // 3. Tared load: baseline 25 kg + 9 kg = 34 kg stays ARMED; +10 kg delta >= 200 ms -> STOPPED.
  SafetyFsm fsm3;
  arm(fsm3, 0, 25.0f);
  in = make_nominal_input(100, 34.0f);
  in.keepalive_pulse = true;
  EXPECT_EQ(fsm3.step(in).state, C8State::kArmed);

  in = make_nominal_input(200, 35.0f);  // +10.0 kg delta
  EXPECT_EQ(fsm3.step(in).state, C8State::kArmed);
  in = make_nominal_input(400, 35.0f);
  out = fsm3.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonLoadDelta) != 0);
}

TEST(C8SafetyTest, InvalidNanOrStaleSensorsAreFailSafe) {
  SafetyFsm fsm;

  // NaN tilt at SAFE_OFF blocks arming even with start edge + RUN + keepalive.
  SafetyInput in = make_nominal_input(100);
  in.keepalive_pulse = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.manual_start_at_raft = true;
  in.tilt.roll_deg = std::numeric_limits<float>::quiet_NaN();
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kSafeOff);
  EXPECT_TRUE(out.sensor_fault);
  EXPECT_FALSE(out.relay_closed);

  // Arm cleanly with valid sensors.
  in = make_nominal_input(200);
  in.manual_start_at_raft = false;
  EXPECT_EQ(fsm.step(in).state, C8State::kSafeOff);

  in = make_nominal_input(300);
  in.keepalive_pulse = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.manual_start_at_raft = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Stale load sensor (> 500 ms old) while ARMED -> fail-safe STOPPED.
  in = make_nominal_input(1000);
  in.load.timestamp_ms = 200;  // 800 ms stale
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_TRUE(out.sensor_fault);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonSensorFault) != 0);
}

TEST(C8SafetyTest, LoraPersistenceAcrossC8AndTransmitterRebootsMultiSenderAndLongKey) {
  RamCounterStorage c8_flash;
  uint8_t long_key[80]{};
  for (size_t i = 0; i < sizeof(long_key); ++i) {
    long_key[i] = static_cast<uint8_t>(0xA0U + (i & 0x1FU));
  }

  uint8_t pkt[kLoraPacketSize]{};
  LoraPacket parsed;

  // Boot 1: GCS (sender 0) sends counter 1; Handheld (sender 1) sends counter 5.
  {
    LoraVerifier v1(kGroupId, long_key, sizeof(long_key), 0U, &c8_flash);
    encode_lora_packet_from_sender(
        LoraCmd::kRun, 0U, kGroupId, 1U, long_key, sizeof(long_key), pkt);
    ASSERT_TRUE(v1.verify_and_parse(pkt, parsed));
    EXPECT_EQ(parsed.sender_id, 0U);
    EXPECT_EQ(parsed.cmd, LoraCmd::kRun);

    encode_lora_packet_from_sender(
        LoraCmd::kHold, 1U, kGroupId, 5U, long_key, sizeof(long_key), pkt);
    ASSERT_TRUE(v1.verify_and_parse(pkt, parsed));
    EXPECT_EQ(parsed.sender_id, 1U);
    EXPECT_EQ(parsed.cmd, LoraCmd::kHold);
  }

  // Boot 2: C8 reboots with fresh LoraVerifier backed by persistent storage.
  LoraVerifier v2(kGroupId, long_key, sizeof(long_key), 0U, &c8_flash);

  // Old captured GCS packet (counter 1) MUST be rejected after C8 reboot!
  encode_lora_packet_from_sender(
      LoraCmd::kRun, 0U, kGroupId, 1U, long_key, sizeof(long_key), pkt);
  EXPECT_FALSE(v2.verify_and_parse(pkt, parsed));

  // Tampered HMAC rejected & wrong group rejected.
  encode_lora_packet_from_sender(
      LoraCmd::kStop, 0U, kGroupId, 2U, long_key, sizeof(long_key), pkt);
  pkt[11] ^= 0x01U;
  EXPECT_FALSE(v2.verify_and_parse(pkt, parsed));
  EXPECT_EQ(v2.last_valid_counter(0U), 1U);

  // Transmitter reboot with persisted boot jump (+1000) succeeds immediately for STOP.
  const uint32_t tx_boot_ctr = advance_boot_counter(1U, 1000U);
  encode_lora_packet_from_sender(
      LoraCmd::kStop, 0U, kBroadcastGroupId, tx_boot_ctr, long_key, sizeof(long_key), pkt);
  ASSERT_TRUE(v2.verify_and_parse(pkt, parsed));
  EXPECT_EQ(parsed.cmd, LoraCmd::kStop);
  EXPECT_EQ(parsed.counter, 1001U);
}

TEST(C8SafetyTest, FourGStopAndWatchdogStopIgnoreBridgeZone) {
  auto arm_with_bz = [](SafetyFsm& fsm) {
    SafetyInput in = make_nominal_input(0);
    in.keepalive_pulse = true;
    in.bridge_zone_refresh = true;
    in.has_lora_cmd = true;
    in.lora_cmd = LoraCmd::kRun;
    in.manual_start_at_raft = true;
    ASSERT_EQ(fsm.step(in).state, C8State::kArmed);
  };

  SafetyFsm fsm_4g;
  arm_with_bz(fsm_4g);
  SafetyInput in = make_nominal_input(500);
  in.stop_from_4g = true;
  SafetyOutput out = fsm_4g.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReason4gStop) != 0);

  SafetyFsm fsm_wd;
  arm_with_bz(fsm_wd);
  in = make_nominal_input(500);
  in.stop_from_watchdog = true;
  out = fsm_wd.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonWatchdogStop) != 0);
}

TEST(C8SafetyTest, KeepaliveLossThreeSecondsTriggersHoldReqAndRecovers) {
  SafetyFsm fsm;
  SafetyInput in = make_nominal_input(1000);
  in.keepalive_pulse = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.manual_start_at_raft = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Under 3 s -> still ARMED.
  in = make_nominal_input(3990);
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);

  // Exactly 3000 ms elapsed -> HOLD_REQ (relay stays closed, HOLD_REQ asserted).
  in = make_nominal_input(4000);
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kHoldReq);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_TRUE(out.hold_req_output);

  // Keepalive resumes -> recovers to ARMED.
  in = make_nominal_input(4100);
  in.keepalive_pulse = true;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);
  EXPECT_FALSE(out.hold_req_output);
}

TEST(C8SafetyTest, BridgeZoneSuppressesTiltLoadUpTo60sOnlyNeverHardStops) {
  SafetyFsm fsm;
  SafetyInput in = make_nominal_input(0, 25.0f);
  in.keepalive_pulse = true;
  in.bridge_zone_refresh = true;
  in.has_lora_cmd = true;
  in.lora_cmd = LoraCmd::kRun;
  in.manual_start_at_raft = true;
  ASSERT_EQ(fsm.step(in).state, C8State::kArmed);

  // Within 60 s BRIDGE_ZONE: load delta >= 10 kg and tilt > 15 deg are suppressed.
  in = make_nominal_input(2000, 38.0f);
  in.keepalive_pulse = true;
  in.tilt.roll_deg = 20.0f;
  SafetyOutput out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.bridge_zone_active);

  in = make_nominal_input(2300, 38.0f);
  in.keepalive_pulse = true;
  in.tilt.roll_deg = 20.0f;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kArmed);
  EXPECT_TRUE(out.relay_closed);

  // Beyond 60 s without refresh -> auto-cutoffs engage immediately -> STOPPED.
  in = make_nominal_input(60001, 38.0f);
  in.keepalive_pulse = true;
  in.tilt.roll_deg = 20.0f;
  out = fsm.step(in);
  EXPECT_EQ(out.state, C8State::kStopped);
  EXPECT_FALSE(out.bridge_zone_active);
  EXPECT_FALSE(out.relay_closed);
  EXPECT_TRUE((out.stop_reason_flags & kReasonLoadDelta) != 0);
  EXPECT_TRUE((out.stop_reason_flags & kReasonDynamicTilt) != 0);
}

TEST(C8SafetyTest, WatchdogNotKickedWhenHalUnhealthy) {
  MockC8Hal hal;
  SafetyFsm fsm;
  LoraVerifier lora(kGroupId, kSharedKey, sizeof(kSharedKey));

  hal.now_ms = 100;
  hal.tilt.timestamp_ms = 100;
  hal.load.timestamp_ms = 100;
  hal.health_ok = true;
  (void)step_hal_cycle(hal, fsm, lora);
  EXPECT_EQ(hal.watchdog_kicks, 1U);

  hal.now_ms = 110;
  hal.tilt.timestamp_ms = 110;
  hal.load.timestamp_ms = 110;
  hal.health_ok = false;
  (void)step_hal_cycle(hal, fsm, lora);
  EXPECT_EQ(hal.watchdog_kicks, 1U);
}

TEST(C8SafetyTest, ThirtyConsecutiveEstopTrialsHundredPercentCutoffViaHal) {
  MockC8Hal hal;
  SafetyFsm fsm;
  RamCounterStorage storage;
  LoraVerifier lora(kGroupId, kSharedKey, sizeof(kSharedKey), 0U, &storage);
  uint32_t lora_counter = 1;
  uint32_t estop_success_count = 0;

  for (int trial = 0; trial < 30; ++trial) {
    // 1. Release buttons & send fresh LoRa RUN, then press start button (rising edge).
    hal.manual_estop = false;
    hal.manual_reset = false;
    hal.manual_start = false;
    hal.stop_4g = false;
    hal.stop_watchdog = false;
    hal.now_ms += 50;
    hal.tilt = TiltSample{0.0f, 0.0f, hal.now_ms, true};
    hal.load = LoadSample{25.0f, hal.now_ms, true};
    hal.keepalive_pulse = true;
    hal.bridge_zone_refresh = (trial % 3 == 0);

    std::array<uint8_t, kLoraPacketSize> run_pkt{};
    encode_lora_packet(
        LoraCmd::kRun, kGroupId, lora_counter++, kSharedKey, sizeof(kSharedKey), run_pkt.data());
    hal.lora_rx.push(run_pkt);
    ASSERT_EQ(step_hal_cycle(hal, fsm, lora).state, C8State::kSafeOff);

    hal.now_ms += 50;
    hal.tilt.timestamp_ms = hal.now_ms;
    hal.load.timestamp_ms = hal.now_ms;
    hal.manual_start = true;
    SafetyOutput out = step_hal_cycle(hal, fsm, lora);
    ASSERT_EQ(out.state, C8State::kArmed) << "Trial " << trial << " failed to arm";
    ASSERT_TRUE(hal.relay_closed);

    // 2. Alternate 10 manual E-stop, 10 LoRa STOP, and 10 4G STOP.
    hal.manual_start = false;
    hal.now_ms += 20;
    hal.tilt.timestamp_ms = hal.now_ms;
    hal.load.timestamp_ms = hal.now_ms;
    if (trial % 3 == 0) {
      hal.manual_estop = true;
    } else if (trial % 3 == 1) {
      std::array<uint8_t, kLoraPacketSize> stop_pkt{};
      encode_lora_packet(
          LoraCmd::kStop, kGroupId, lora_counter++, kSharedKey, sizeof(kSharedKey), stop_pkt.data());
      hal.lora_rx.push(stop_pkt);
    } else {
      hal.stop_4g = true;
    }

    out = step_hal_cycle(hal, fsm, lora);
    if (out.state == C8State::kStopped && !hal.relay_closed) {
      ++estop_success_count;
    }

    // 3. Clear faults and perform rising-edge manual reset at raft for next trial.
    hal.manual_estop = false;
    hal.stop_4g = false;
    hal.now_ms += 20;
    hal.tilt.timestamp_ms = hal.now_ms;
    hal.load.timestamp_ms = hal.now_ms;
    hal.manual_reset = true;
    out = step_hal_cycle(hal, fsm, lora);
    ASSERT_EQ(out.state, C8State::kSafeOff);
    ASSERT_FALSE(hal.relay_closed);
  }

  EXPECT_EQ(estop_success_count, 30U);
  EXPECT_EQ(hal.watchdog_kicks, 120U);
}

}  // namespace
