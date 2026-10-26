#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <algorithm>

namespace bit_stream_handling {

inline std::optional<size_t> scan_stream(const std::vector<bool> &stream, uint16_t pattern, size_t pattern_len, std::optional<size_t> scan_distance = std::nullopt) {
  size_t dist = scan_distance.value_or(stream.size());

  if (stream.size() < pattern_len) return std::nullopt;

  size_t limit = 0;
  if (stream.size() >= pattern_len + dist) {
    limit = stream.size() - pattern_len - dist;
  }

  for (size_t i = 0; i <= limit; i++) {
    uint16_t window = 0;
    for (size_t j = 0; j < pattern_len; j++) {
      window = (window << 1) | (stream[i + j] ? 1 : 0);
    }

    if (window == pattern) {
      return i; // Return index where pattern match starts
    }
  }
  return std::nullopt;
}

inline uint32_t read_bits(const std::vector<bool> &stream, size_t start, size_t len) {
  uint32_t val = 0;
  for (size_t k = 0; k < len; k++) {
    bool bit = stream[start + k];
    val = (val << 1) | (bit ? 1 : 0);
  }
  return val;
}

inline uint16_t calculate_crc16_ccitt(const std::vector<bool> &stream, size_t start, size_t num_bits) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < num_bits; i += 8) {
    uint8_t byte_val = 0;
    size_t chunk = std::min((size_t)8, num_bits - i);
    for (size_t b = 0; b < chunk; b++) {
      byte_val = (byte_val << 1) | (stream[start + i + b] ? 1 : 0);
    }
    crc ^= (uint16_t)byte_val << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc ^ 0xFFFF; // Apply final inversion (XOR out) to match Genibus / SCM+ spec
}

}  // namespace bit_stream_handling
