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
      // if (raw_bits.size() > 40) break; 
      // raw_bits.clear();
      // continue;
      break;
    }

    float bit_count_fraction = (float)duration / bit_period_us;
    int bit_multiplier = (int)std::round(bit_count_fraction);

    for (int b = 0; b < bit_multiplier; b++) {
      raw_bits.push_back(bit_value);
    }
  }
  return raw_bits;
}

/**
 * Decodes a raw ESPHome remote receiver pulse train into bits, returning a std::vector<bool>
 * based on a target baud rate, using an adaptive, forward-looking error correction mechanism.
 * 
 * @param src The source vector containing ESPHome remote_base::RemoteReceiveData
 * @param baud_rate The target transmission rate (e.g., 2400, 9600)
 * @param tolerance_pct The maximum allowed deviation (0.0 to 1.0) before triggering a correction attempt
 * @return A std::vector<bool> containing the decoded bits. Returns an empty vector if an unrecoverable timing drift occurs.
 */
std::vector<bool> decode_pulses_to_bits(const std::vector<int32_t>& src, uint32_t baud_rate, float tolerance_pct) {
    std::vector<bool> out_bits;
    if (src.empty() || baud_rate == 0) return out_bits;

    // Calculate the duration of a single bit/baud period in microseconds
    double bit_period_us = 1000000.0 / baud_rate;

    size_t i = 0;
    while (i < src.size()) {
        // ESPHome uses negative values for SPACE (idle) and positive for MARK (pulse)
        int32_t raw_duration = std::abs(src[i]);
        
        // Calculate how many bit periods this pulse represents
        double raw_bits = (double)raw_duration / bit_period_us;
        int32_t nominal_bits = std::round(raw_bits);
        
        if (nominal_bits == 0) {
            nominal_bits = 1; // Safeguard against sub-period glitches
        }

        // Calculate deviation from the nearest perfect integer bit period
        double ideal_duration = nominal_bits * bit_period_us;
        double deviation = std::abs((double)raw_duration - ideal_duration) / ideal_duration;

        // Check if the pulse exceeds our strict tolerance threshold
        if (deviation > tolerance_pct) {
            // Look-ahead Look: See if the current pulse error is offset by the next pulse's error
            if (i + 1 < src.size()) {
                int32_t next_raw = std::abs(src[i + 1]);
                int32_t combined_duration = raw_duration + next_raw;
                
                // Calculate nominal bits for the combined pair window
                double combined_raw_bits = (double)combined_duration / bit_period_us;
                int32_t combined_nominal_bits = std::round(combined_raw_bits);
                
                double combined_ideal = combined_nominal_bits * bit_period_us;
                double combined_deviation = std::abs((double)combined_duration - combined_ideal) / combined_ideal;

                // If combining them eliminates or drastically reduces the drift, perform the repair
                if (combined_deviation <= tolerance_pct) {
                    // Recalculate current nominal bits using the verified combined context
                    double adjusted_raw_bits = combined_ideal - next_raw;
                    nominal_bits = std::round(adjusted_raw_bits / bit_period_us);
                    if (nominal_bits <= 0) nominal_bits = 1;
                } else {
                    // Drift unrecoverable via simple look-ahead window; return empty vector to signal failure
                    out_bits.clear();
                    return out_bits; 
                }
            } else {
                // End of stream reached, cannot look ahead to fix the drift
                out_bits.clear();
                return out_bits;
            }
        }

        // Determine bit polarity based on the ESPHome signal type
        // Positive = MARK (true), Negative = SPACE (false)
        bool bit_value = (src[i] > 0);

        // Push the decoded bits into the optimized std::vector<bool>
        for (int32_t b = 0; b < nominal_bits; ++b) {
            out_bits.push_back(bit_value);
        }

        i++;
    }

    return out_bits;
}

}  // namespace remote_receiver_helpers
