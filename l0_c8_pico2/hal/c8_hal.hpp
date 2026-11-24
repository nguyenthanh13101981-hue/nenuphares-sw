#ifndef NENUPHAR_L0_C8_HAL_HPP_
#define NENUPHAR_L0_C8_HAL_HPP_

#include <cstdint>

namespace nenuphar::c8 {

struct TiltSample {
  float roll_deg{0.0f};
  float pitch_deg{0.0f};
  uint32_t timestamp_ms{0};
  bool valid{false};
};

struct LoadSample {
  float total_load_kg{0.0f};
  uint32_t timestamp_ms{0};
  bool valid{false};
};

/**
 * Hardware Abstraction Layer for safety board C8 (Raspberry Pi Pico 2).
 * Keeps bare-metal SDK / GPIO / SPI / I2C drivers decoupled from core/.
 */
class C8Hal {
 public:
  virtual ~C8Hal() = default;

  virtual uint32_t millis() const = 0;
  virtual bool read_manual_estop() const = 0;
  virtual bool read_manual_reset_button() const = 0;
  virtual bool read_start_button() const = 0;        // Physical start button at raft (A.10)
  virtual bool read_4g_stop() const = 0;             // 4G emergency STOP via C4.2
  virtual bool read_watchdog_stop() const = 0;       // Watchdog hard STOP signal
  virtual TiltSample read_tilt_sample() const = 0;   // MPU-6050 (>= 50 Hz) with validity
  virtual LoadSample read_load_sample() const = 0;   // 4x HX711 load cells with validity
  virtual bool poll_keepalive_pulse() = 0;           // C4 -> C8 1 Hz keepalive
  virtual bool poll_bridge_zone_refresh() = 0;       // C4 -> C8 BRIDGE_ZONE <= 60 s
  virtual bool poll_lora_packet(uint8_t out_pkt[12]) = 0;  // SX1262 12-byte frame
  virtual bool cycle_health_ok() const = 0;          // True only if bus/sensor reads completed

  virtual void set_propulsion_relay(bool closed) = 0;
  virtual void set_hold_req_output(bool active) = 0;
  virtual void kick_external_watchdog() = 0;         // 100 ms external HW watchdog
};

}  // namespace nenuphar::c8

#endif  // NENUPHAR_L0_C8_HAL_HPP_
