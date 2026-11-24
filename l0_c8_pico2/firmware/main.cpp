#include <cstdint>

#include "core/lora_protocol.hpp"
#include "core/safety_fsm.hpp"
#include "firmware/pico_hal.hpp"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif

namespace {

constexpr uint16_t kLocalGroupId = 0x0101U;
// Provision per-raft pre-shared key during HIL bring-up.
constexpr uint8_t kProvisionedKey[16] = {
    0x4E, 0x45, 0x4E, 0x55, 0x50, 0x48, 0x41, 0x52,
    0x42, 0x4F, 0x4E, 0x4E, 0x45, 0x56, 0x41, 0x4C,
};
constexpr uint32_t kControlLoopPeriodMs = 10U;  // 100 Hz (A.10)

}  // namespace

int main() {
  using namespace nenuphar::c8;

  firmware::PicoC8Hal hal;
  hal.init_peripherals();

  firmware::PicoFlashCounterStorage storage;
  LoraVerifier lora(kLocalGroupId, kProvisionedKey, sizeof(kProvisionedKey), 0U, &storage);
  SafetyFsm fsm;

  while (true) {
    const uint32_t cycle_start_ms = hal.millis();
    (void)step_hal_cycle(hal, fsm, lora);
#ifdef PICO_ON_DEVICE
    const uint32_t elapsed_ms = hal.millis() - cycle_start_ms;
    if (elapsed_ms < kControlLoopPeriodMs) {
      sleep_ms(kControlLoopPeriodMs - elapsed_ms);
    }
#else
    (void)cycle_start_ms;
    (void)kControlLoopPeriodMs;
    break;
#endif
  }

  return 0;
}
