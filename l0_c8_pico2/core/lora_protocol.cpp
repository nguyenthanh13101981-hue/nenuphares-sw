#include "core/lora_protocol.hpp"

#include <cstring>

namespace nenuphar::c8 {
namespace {

constexpr uint32_t kSha256K[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
};

inline uint32_t rotr(uint32_t x, uint32_t n) {
  return (x >> n) | (x << (32U - n));
}

class Sha256 {
 public:
  Sha256() { reset(); }

  void reset() {
    state_[0] = 0x6a09e667U;
    state_[1] = 0xbb67ae85U;
    state_[2] = 0x3c6ef372U;
    state_[3] = 0xa54ff53aU;
    state_[4] = 0x510e527fU;
    state_[5] = 0x9b05688cU;
    state_[6] = 0x1f83d9abU;
    state_[7] = 0x5be0cd19U;
    bit_len_ = 0;
    buf_len_ = 0;
  }

  void update(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      buf_[buf_len_++] = data[i];
      if (buf_len_ == 64) {
        transform(buf_);
        bit_len_ += 512;
        buf_len_ = 0;
      }
    }
  }

  void finish(uint8_t out[32]) {
    const uint64_t total_bits = bit_len_ + static_cast<uint64_t>(buf_len_) * 8U;
    buf_[buf_len_++] = 0x80U;
    if (buf_len_ > 56) {
      while (buf_len_ < 64) {
        buf_[buf_len_++] = 0x00U;
      }
      transform(buf_);
      buf_len_ = 0;
    }
    while (buf_len_ < 56) {
      buf_[buf_len_++] = 0x00U;
    }
    for (int i = 7; i >= 0; --i) {
      buf_[56 + (7 - i)] = static_cast<uint8_t>((total_bits >> (i * 8)) & 0xFFU);
    }
    transform(buf_);
    for (size_t i = 0; i < 8; ++i) {
      out[i * 4 + 0] = static_cast<uint8_t>((state_[i] >> 24) & 0xFFU);
      out[i * 4 + 1] = static_cast<uint8_t>((state_[i] >> 16) & 0xFFU);
      out[i * 4 + 2] = static_cast<uint8_t>((state_[i] >> 8) & 0xFFU);
      out[i * 4 + 3] = static_cast<uint8_t>(state_[i] & 0xFFU);
    }
  }

 private:
  void transform(const uint8_t block[64]) {
    uint32_t w[64];
    for (size_t i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(block[i * 4 + 0]) << 24) |
             (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
             (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
             static_cast<uint32_t>(block[i * 4 + 3]);
    }
    for (size_t i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t a = state_[0];
    uint32_t b = state_[1];
    uint32_t c = state_[2];
    uint32_t d = state_[3];
    uint32_t e = state_[4];
    uint32_t f = state_[5];
    uint32_t g = state_[6];
    uint32_t h = state_[7];

    for (size_t i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = h + s1 + ch + kSha256K[i] + w[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = s0 + maj;

      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  uint32_t state_[8]{};
  uint64_t bit_len_{0};
  uint8_t buf_[64]{};
  size_t buf_len_{0};
};

bool is_valid_cmd(uint8_t raw) {
  return raw == static_cast<uint8_t>(LoraCmd::kRun) ||
         raw == static_cast<uint8_t>(LoraCmd::kStop) ||
         raw == static_cast<uint8_t>(LoraCmd::kHold);
}

}  // namespace

uint32_t RamCounterStorage::load_counter(uint8_t sender_id) const {
  if (sender_id >= kMaxLoraSenders) {
    return 0U;
  }
  return counters_[sender_id];
}

void RamCounterStorage::save_counter(uint8_t sender_id, uint32_t counter) {
  if (sender_id < kMaxLoraSenders) {
    counters_[sender_id] = counter;
  }
}

uint32_t advance_boot_counter(uint32_t stored_counter, uint32_t jump) {
  const uint64_t next = static_cast<uint64_t>(stored_counter) + static_cast<uint64_t>(jump);
  if (next > 0xFFFFFFFFULL) {
    return 0xFFFFFFFFU;
  }
  return static_cast<uint32_t>(next);
}

void compute_hmac_sha256(
    const uint8_t* key,
    size_t key_len,
    const uint8_t* data,
    size_t data_len,
    uint8_t out_digest[32]) {
  uint8_t key_block[64]{};
  if (key != nullptr && key_len > 64) {
    Sha256 hk;
    hk.update(key, key_len);
    hk.finish(key_block);
  } else if (key != nullptr && key_len > 0) {
    std::memcpy(key_block, key, key_len);
  }

  uint8_t ipad[64];
  uint8_t opad[64];
  for (size_t i = 0; i < 64; ++i) {
    ipad[i] = static_cast<uint8_t>(key_block[i] ^ 0x36U);
    opad[i] = static_cast<uint8_t>(key_block[i] ^ 0x5cU);
  }

  uint8_t inner_hash[32];
  Sha256 inner;
  inner.update(ipad, 64);
  if (data != nullptr && data_len > 0) {
    inner.update(data, data_len);
  }
  inner.finish(inner_hash);

  Sha256 outer;
  outer.update(opad, 64);
  outer.update(inner_hash, 32);
  outer.finish(out_digest);
}

void encode_lora_packet_from_sender(
    LoraCmd cmd,
    uint8_t sender_id,
    uint16_t group_id,
    uint32_t counter,
    const uint8_t* key,
    size_t key_len,
    uint8_t out_packet[kLoraPacketSize]) {
  const uint8_t masked_sender = static_cast<uint8_t>(sender_id & 0x0FU);
  const uint8_t masked_cmd = static_cast<uint8_t>(static_cast<uint8_t>(cmd) & 0x0FU);
  out_packet[0] = static_cast<uint8_t>((masked_sender << 4) | masked_cmd);
  out_packet[1] = static_cast<uint8_t>((group_id >> 8) & 0xFFU);
  out_packet[2] = static_cast<uint8_t>(group_id & 0xFFU);
  out_packet[3] = static_cast<uint8_t>((counter >> 24) & 0xFFU);
  out_packet[4] = static_cast<uint8_t>((counter >> 16) & 0xFFU);
  out_packet[5] = static_cast<uint8_t>((counter >> 8) & 0xFFU);
  out_packet[6] = static_cast<uint8_t>(counter & 0xFFU);

  uint8_t digest[32];
  compute_hmac_sha256(key, key_len, out_packet, 7, digest);
  std::memcpy(out_packet + 7, digest, 5);
}

void encode_lora_packet(
    LoraCmd cmd,
    uint16_t group_id,
    uint32_t counter,
    const uint8_t* key,
    size_t key_len,
    uint8_t out_packet[kLoraPacketSize]) {
  encode_lora_packet_from_sender(
      cmd, 0U, group_id, counter, key, key_len, out_packet);
}

LoraVerifier::LoraVerifier(
    uint16_t local_group_id,
    const uint8_t* key,
    size_t key_len,
    uint32_t initial_counter,
    CounterStorage* storage)
    : local_group_id_(local_group_id), storage_(storage) {
  if (key != nullptr && key_len > kMaxKeyLen) {
    // Match RFC 2104 key reduction inside compute_hmac_sha256 for keys > 64 bytes.
    Sha256 hk;
    hk.update(key, key_len);
    hk.finish(key_);
    key_len_ = 32U;
  } else if (key != nullptr && key_len > 0) {
    key_len_ = key_len;
    std::memcpy(key_, key, key_len_);
  }

  for (size_t i = 0; i < kMaxLoraSenders; ++i) {
    uint32_t c = initial_counter;
    if (storage_ != nullptr) {
      const uint32_t persisted = storage_->load_counter(static_cast<uint8_t>(i));
      if (persisted > c) {
        c = persisted;
      }
    }
    last_valid_counters_[i] = c;
  }
}

uint32_t LoraVerifier::last_valid_counter(uint8_t sender_id) const {
  if (sender_id >= kMaxLoraSenders) {
    return 0U;
  }
  return last_valid_counters_[sender_id];
}

bool LoraVerifier::verify_and_parse(const uint8_t pkt[kLoraPacketSize], LoraPacket& out) {
  if (pkt == nullptr) {
    return false;
  }

  const uint8_t sender_id = static_cast<uint8_t>((pkt[0] >> 4) & 0x0FU);
  const uint8_t cmd_raw = static_cast<uint8_t>(pkt[0] & 0x0FU);
  if (!is_valid_cmd(cmd_raw)) {
    return false;
  }

  const uint16_t group_id =
      (static_cast<uint16_t>(pkt[1]) << 8) | static_cast<uint16_t>(pkt[2]);
  if (group_id != local_group_id_ && group_id != kBroadcastGroupId) {
    return false;
  }

  const uint32_t counter = (static_cast<uint32_t>(pkt[3]) << 24) |
                           (static_cast<uint32_t>(pkt[4]) << 16) |
                           (static_cast<uint32_t>(pkt[5]) << 8) |
                           static_cast<uint32_t>(pkt[6]);
  if (counter <= last_valid_counters_[sender_id]) {
    return false;
  }

  uint8_t expected[32];
  compute_hmac_sha256(key_, key_len_, pkt, 7, expected);

  uint8_t diff = 0;
  for (size_t i = 0; i < 5; ++i) {
    diff |= static_cast<uint8_t>(pkt[7 + i] ^ expected[i]);
  }
  if (diff != 0) {
    return false;
  }

  last_valid_counters_[sender_id] = counter;
  if (storage_ != nullptr) {
    storage_->save_counter(sender_id, counter);
  }
  out.cmd = static_cast<LoraCmd>(cmd_raw);
  out.sender_id = sender_id;
  out.group_id = group_id;
  out.counter = counter;
  return true;
}

}  // namespace nenuphar::c8
