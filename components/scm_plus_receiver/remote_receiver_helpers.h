#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include "esphome/components/remote_base/remote_base.h"

namespace remote_receiver_helpers {

inline std::vector<bool> extract_raw_bits(const esphome::remote_base::RemoteReceiveData &data, size_t reserve_size = 512, float bit_period_us = 31.0f, int32_t max_noise_duration_us = 300) {
  std::vector<bool> raw_bits;
  raw_bits.reserve(reserve_size);

  for (size_t i = 0; i < data.size(); i++) {
    int32_t duration_raw = data[i];
    int32_t duration = std::abs(duration_raw);
    bool bit_value = (duration_raw > 0); 

    if (duration > max_noise_duration_us) {
      if (raw_bits.size() > 40) break; 
      raw_bits.clear();
      continue;
    }

    float bit_count_fraction = (float)duration / bit_period_us;
    int bit_multiplier = (int)std::round(bit_count_fraction);

    for (int b = 0; b < bit_multiplier; b++) {
      raw_bits.push_back(bit_value);
    }
  }
  return raw_bits;
}

}  // namespace remote_receiver_helpers
