#include "firmware/pico_hal.hpp"

#ifdef PICO_ON_DEVICE
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#endif

namespace nenuphar::c8::firmware {

uint32_t PicoFlashCounterStorage::load_counter(uint8_t sender_id) const {
  if (sender_id >= kMaxLoraSenders) {
    return 0U;
  }
  return shadow_[sender_id];
}

void PicoFlashCounterStorage::save_counter(uint8_t sender_id, uint32_t counter) {
  if (sender_id < kMaxLoraSenders) {
    shadow_[sender_id] = counter;
  }
}

PicoC8Hal::PicoC8Hal(const PicoPinMap& pins) : pins_(pins) {}

void PicoC8Hal::init_peripherals() {
#ifdef PICO_ON_DEVICE
  stdio_init_all();
  const uint8_t inputs[] = {
      pins_.gpio_manual_estop,
      pins_.gpio_manual_reset,
      pins_.gpio_manual_start,
      pins_.gpio_4g_stop,
      pins_.gpio_watchdog_stop,
      pins_.gpio_keepalive_in,
      pins_.gpio_bridge_zone_in,
  };
  for (uint8_t pin : inputs) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
  }
  const uint8_t outputs[] = {
      pins_.gpio_propulsion_relay,
      pins_.gpio_hold_req_out,
      pins_.gpio_ext_wd_kick,
  };
  for (uint8_t pin : outputs) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
  }
  initialized_ = true;
#else
  (void)pins_;
  initialized_ = false;
#endif
}

uint32_t PicoC8Hal::millis() const {
#ifdef PICO_ON_DEVICE
  return to_ms_since_boot(get_absolute_time());
#else
  return 0U;
#endif
}

bool PicoC8Hal::read_manual_estop() const {
#ifdef PICO_ON_DEVICE
  // Active-low fail-safe wiring: open circuit or pressed -> true (E-stop engaged).
  return !gpio_get(pins_.gpio_manual_estop);
#else
  return true;
#endif
}

bool PicoC8Hal::read_manual_reset_button() const {
#ifdef PICO_ON_DEVICE
  return !gpio_get(pins_.gpio_manual_reset);
#else
  return false;
#endif
}

bool PicoC8Hal::read_start_button() const {
#ifdef PICO_ON_DEVICE
  return !gpio_get(pins_.gpio_manual_start);
#else
  return false;
#endif
}

bool PicoC8Hal::read_4g_stop() const {
#ifdef PICO_ON_DEVICE
  return gpio_get(pins_.gpio_4g_stop);
#else
  return false;
#endif
}

bool PicoC8Hal::read_watchdog_stop() const {
#ifdef PICO_ON_DEVICE
  return gpio_get(pins_.gpio_watchdog_stop);
#else
  return false;
#endif
}

TiltSample PicoC8Hal::read_tilt_sample() const {
  // Pending MPU-6050 I2C driver integration (A.14 §5); fail-safe invalid until bound.
  return TiltSample{};
}

LoadSample PicoC8Hal::read_load_sample() const {
  // Pending 4x HX711 PIO driver integration (A.14 §5); fail-safe invalid until bound.
  return LoadSample{};
}

bool PicoC8Hal::poll_keepalive_pulse() {
#ifdef PICO_ON_DEVICE
  return gpio_get(pins_.gpio_keepalive_in);
#else
  return false;
#endif
}

bool PicoC8Hal::poll_bridge_zone_refresh() {
#ifdef PICO_ON_DEVICE
  return gpio_get(pins_.gpio_bridge_zone_in);
#else
  return false;
#endif
}

bool PicoC8Hal::poll_lora_packet(uint8_t out_pkt[12]) {
  (void)out_pkt;
  // Pending SX1262 SPI driver integration (A.14 §5).
  return false;
}

bool PicoC8Hal::cycle_health_ok() const {
  return initialized_;
}

void PicoC8Hal::set_propulsion_relay(bool closed) {
#ifdef PICO_ON_DEVICE
  gpio_put(pins_.gpio_propulsion_relay, closed);
#else
  (void)closed;
#endif
}

void PicoC8Hal::set_hold_req_output(bool active) {
#ifdef PICO_ON_DEVICE
  gpio_put(pins_.gpio_hold_req_out, active);
#else
  (void)active;
#endif
}

void PicoC8Hal::kick_external_watchdog() {
#ifdef PICO_ON_DEVICE
  gpio_xor_mask(1U << pins_.gpio_ext_wd_kick);
#endif
}

}  // namespace nenuphar::c8::firmware
