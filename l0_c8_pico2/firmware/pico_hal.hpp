#ifndef NENUPHAR_L0_C8_FIRMWARE_PICO_HAL_HPP_
#define NENUPHAR_L0_C8_FIRMWARE_PICO_HAL_HPP_

#include <cstdint>

#include "core/lora_protocol.hpp"
#include "hal/c8_hal.hpp"

namespace nenuphar::c8::firmware {

/** RP2350 Pico 2 pinout assignment (placeholder pending C8 PCB freeze per A.14 §5). */
struct PicoPinMap {
  uint8_t gpio_manual_estop{2};
  uint8_t gpio_manual_reset{3};
  uint8_t gpio_manual_start{4};
  uint8_t gpio_4g_stop{5};
  uint8_t gpio_watchdog_stop{6};
  uint8_t gpio_keepalive_in{7};
  uint8_t gpio_bridge_zone_in{8};
  uint8_t gpio_propulsion_relay{9};
  uint8_t gpio_hold_req_out{10};
  uint8_t gpio_ext_wd_kick{11};
};

/**
 * RP2350 Flash-backed anti-replay counter storage skeleton.
 * On real Pico 2 hardware, map each sender slot to a wear-leveled flash sector.
 */
class PicoFlashCounterStorage : public CounterStorage {
 public:
  uint32_t load_counter(uint8_t sender_id) const override;
  void save_counter(uint8_t sender_id, uint32_t counter) override;

 private:
  uint32_t shadow_[kMaxLoraSenders]{};
};

/**
 * Pico 2 HAL skeleton implementing C8Hal interface for RP2350 bare-metal builds.
 * Hardware peripheral drivers (MPU-6050 I2C, 4x HX711 PIO, SX1262 SPI) must be
 * bound here once C8 PCB schematic is frozen.
 */
class PicoC8Hal : public C8Hal {
 public:
  explicit PicoC8Hal(const PicoPinMap& pins = PicoPinMap{});

  void init_peripherals();

  uint32_t millis() const override;
  bool read_manual_estop() const override;
  bool read_manual_reset_button() const override;
  bool read_start_button() const override;
  bool read_4g_stop() const override;
  bool read_watchdog_stop() const override;
  TiltSample read_tilt_sample() const override;
  LoadSample read_load_sample() const override;
  bool poll_keepalive_pulse() override;
  bool poll_bridge_zone_refresh() override;
  bool poll_lora_packet(uint8_t out_pkt[12]) override;
  bool cycle_health_ok() const override;

  void set_propulsion_relay(bool closed) override;
  void set_hold_req_output(bool active) override;
  void kick_external_watchdog() override;

 private:
  PicoPinMap pins_;
  bool initialized_{false};
};

}  // namespace nenuphar::c8::firmware

#endif  // NENUPHAR_L0_C8_FIRMWARE_PICO_HAL_HPP_
