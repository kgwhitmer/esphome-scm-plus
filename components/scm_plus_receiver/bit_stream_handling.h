#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <algorithm>

namespace bit_stream_handling {

inline uint32_t read_bits(const std::vector<bool> &stream, size_t start, size_t len) {
  uint32_t val = 0;
  for (size_t k = 0; k < len; k++) {
    bool bit = stream[start + k];
    val = (val << 1) | (bit ? 1 : 0);
  }
  return val;
}

inline std::optional<size_t> scan_stream(const std::vector<bool> &stream, uint16_t pattern, size_t pattern_len, std::optional<size_t> scan_distance = std::nullopt, uint8_t bit_errs = 0) {
  size_t dist = scan_distance.value_or(stream.size());

  if (stream.size() < pattern_len) return std::nullopt;

  size_t limit = 0;
  if (stream.size() >= pattern_len + dist) {
    limit = stream.size() - pattern_len - dist;
  }

  for (size_t i = 0; i <= limit; i++) {
    uint32_t window = read_bits(stream, i, pattern_len);
    uint32_t diff = window ^ pattern;
    uint8_t current_errors = __builtin_popcount(diff);

    if (current_errors <= bit_errs) {
      return i; // Return index where pattern match starts
    }
  }
  return std::nullopt;
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

// Attempts to correct a single-bit error in the specified bit range of the stream.
// Returns true if a single-bit error was found and corrected in-place.
inline bool correct_single_bit_error(std::vector<bool> &stream, size_t start_idx, size_t num_bits, uint16_t expected_crc) {
  for (size_t i = 0; i < num_bits; i++) {
    // Flip the bit at the current position
    stream[start_idx + i] = !stream[start_idx + i];

    // Recompute the CRC for the payload
    uint16_t calc_crc = calculate_crc16_ccitt(stream, start_idx, num_bits);

    // If the CRC matches, we found and corrected the exact single-bit error!
    if (calc_crc == expected_crc) {
      return true;
    }

    // Otherwise, revert the bit and try the next position
    stream[start_idx + i] = !stream[start_idx + i];
  }
  return false;
}

// Attempts to correct a single-bit error using an $O(1)$ syndrome lookup table
inline bool correct_single_bit_error_lookup(std::vector<bool> &stream, size_t start_idx, size_t num_bits, uint16_t expected_crc, uint16_t calculated_crc) {
  static std::unordered_map<uint16_t, size_t> syndrome_table = []() {
    std::unordered_map<uint16_t, size_t> table;
    std::vector<bool> dummy(96, false);
    for (size_t i = 0; i < 96; i++) {
      dummy[i] = true;
      uint16_t crc = calculate_crc16_ccitt(dummy, 0, 96);
      table[crc] = i;
      dummy[i] = false;
    }
    return table;
  }();

  uint16_t syndrome = calculated_crc ^ expected_crc ^ 0xFFFF;
  auto it = syndrome_table.find(syndrome);
  if (it != syndrome_table.end()) {
    size_t bit_idx = it->second;
    stream[start_idx + bit_idx] = !stream[start_idx + bit_idx];
    return true;
  }
  
  return false;
}

}  // namespace bit_stream_handling
