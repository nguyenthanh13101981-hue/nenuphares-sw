#ifndef NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_
#define NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace nenuphar::c8 {

constexpr size_t kLoraPacketSize = 12;
constexpr uint16_t kBroadcastGroupId = 0xFFFFU;

enum class LoraCmd : uint8_t {
  kRun = 0x01,
  kStop = 0x02,
  kHold = 0x03,
};

struct LoraPacket {
  LoraCmd cmd{LoraCmd::kStop};
  uint16_t group_id{0};
  uint32_t counter{0};
};

/** Compute full 32-byte HMAC-SHA256 using pure bare-metal C++ (no heap). */
void compute_hmac_sha256(
    const uint8_t* key,
    size_t key_len,
    const uint8_t* data,
    size_t data_len,
    uint8_t out_digest[32]);

/**
 * Encode 12-byte D-27 LoRa frame:
 *   [0]    CMD (1B)
 *   [1..2] GROUP_ID (2B big-endian)
 *   [3..6] COUNTER (4B big-endian)
 *   [7..11] HMAC_TRUNC (5B)
 */
void encode_lora_packet(
    LoraCmd cmd,
    uint16_t group_id,
    uint32_t counter,
    const uint8_t* key,
    size_t key_len,
    uint8_t out_packet[kLoraPacketSize]);

/** Verifies HMAC_TRUNC, group membership, and strictly monotonic anti-replay counter. */
class LoraVerifier {
 public:
  LoraVerifier(
      uint16_t local_group_id,
      const uint8_t* key,
      size_t key_len,
      uint32_t initial_counter = 0);

  bool verify_and_parse(const uint8_t pkt[kLoraPacketSize], LoraPacket& out);
  uint32_t last_valid_counter() const { return last_valid_counter_; }

 private:
  static constexpr size_t kMaxKeyLen = 64;
  uint16_t local_group_id_;
  uint8_t key_[kMaxKeyLen]{};
  size_t key_len_{0};
  uint32_t last_valid_counter_{0};
};

}  // namespace nenuphar::c8

#endif  // NENUPHAR_L0_C8_LORA_PROTOCOL_HPP_
