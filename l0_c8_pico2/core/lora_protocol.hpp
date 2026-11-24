#ifndef NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_
#define NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace nenuphar::c8 {

constexpr size_t kLoraPacketSize = 12;
constexpr size_t kMaxLoraSenders = 16;
constexpr uint16_t kBroadcastGroupId = 0xFFFFU;
constexpr uint32_t kDefaultBootCounterJump = 1000U;

enum class LoraCmd : uint8_t {
  kRun = 0x01,
  kStop = 0x02,
  kHold = 0x03,
};

struct LoraPacket {
  LoraCmd cmd{LoraCmd::kStop};
  uint8_t sender_id{0};
  uint16_t group_id{0};
  uint32_t counter{0};
};

/** Non-volatile storage abstraction for per-sender anti-replay counters across reboots. */
class CounterStorage {
 public:
  virtual ~CounterStorage() = default;
  virtual uint32_t load_counter(uint8_t sender_id) const = 0;
  virtual void save_counter(uint8_t sender_id, uint32_t counter) = 0;
};

/** Simple in-memory CounterStorage implementation for host unit tests or boot-jump simulation. */
class RamCounterStorage : public CounterStorage {
 public:
  uint32_t load_counter(uint8_t sender_id) const override;
  void save_counter(uint8_t sender_id, uint32_t counter) override;

 private:
  uint32_t counters_[kMaxLoraSenders]{};
};

/** Advance a persisted transmitter counter on boot to guarantee strictly monotonic progress. */
uint32_t advance_boot_counter(
    uint32_t stored_counter,
    uint32_t jump = kDefaultBootCounterJump);

/** Compute full 32-byte HMAC-SHA256 using pure bare-metal C++ (no heap). */
void compute_hmac_sha256(
    const uint8_t* key,
    size_t key_len,
    const uint8_t* data,
    size_t data_len,
    uint8_t out_digest[32]);

/**
 * Encode 12-byte D-27 LoRa frame (sender_id in high nibble of byte 0):
 *   [0]     (sender_id << 4) | cmd
 *   [1..2]  GROUP_ID (2B big-endian)
 *   [3..6]  COUNTER (4B big-endian)
 *   [7..11] HMAC_TRUNC (5B)
 */
void encode_lora_packet_from_sender(
    LoraCmd cmd,
    uint8_t sender_id,
    uint16_t group_id,
    uint32_t counter,
    const uint8_t* key,
    size_t key_len,
    uint8_t out_packet[kLoraPacketSize]);

/** Encode 12-byte D-27 LoRa frame for default sender_id = 0 (GCS). */
void encode_lora_packet(
    LoraCmd cmd,
    uint16_t group_id,
    uint32_t counter,
    const uint8_t* key,
    size_t key_len,
    uint8_t out_packet[kLoraPacketSize]);

/** Verifies HMAC_TRUNC, group membership, and per-sender persistent anti-replay counters. */
class LoraVerifier {
 public:
  LoraVerifier(
      uint16_t local_group_id,
      const uint8_t* key,
      size_t key_len,
      uint32_t initial_counter = 0,
      CounterStorage* storage = nullptr);

  bool verify_and_parse(const uint8_t pkt[kLoraPacketSize], LoraPacket& out);
  uint32_t last_valid_counter(uint8_t sender_id = 0) const;

 private:
  static constexpr size_t kMaxKeyLen = 64;
  uint16_t local_group_id_;
  uint8_t key_[kMaxKeyLen]{};
  size_t key_len_{0};
  uint32_t last_valid_counters_[kMaxLoraSenders]{};
  CounterStorage* storage_{nullptr};
};

}  // namespace nenuphar::c8

#endif  // NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_
