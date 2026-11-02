#pragma once

#include "esphome.h"
#include "esphome/core/application.h"
#include "esphome/core/automation.h"
#include "esphome/components/remote_base/remote_base.h"
#include <cmath> 
#include <vector>
#include <string>
#include <algorithm>

#include "remote_receiver_helpers.h"
#include "bit_stream_handling.h"

// Forward declare LightState so pointer member variable is valid without including the header
namespace esphome::light {
class LightState;
}

#ifdef USE_LIGHT
#include "esphome/components/light/light_state.h"
#endif

#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

namespace scm_plus {

class ScmPlusDecoder : public esphome::Component, public esphome::remote_base::RemoteReceiverListener {
 public:
  esphome::light::LightState *status_led_{nullptr};
  float brightness_scale_{0.10f}; 

  float current_r_{0.0f};
  float current_g_{0.0f};
  float current_b_{0.0f};

  uint32_t receive_call_count_{0};
  std::vector<uint32_t> filter_meter_ids_; 
  std::string custom_base_topic_{""}; 

  esphome::CallbackManager<void(JsonObject)> json_callback_{};

  ScmPlusDecoder(float brightness) : brightness_scale_(brightness) {}

  void setup() override {
    ESP_LOGD("scm_status", "Component initialization / boot");
    this->set_led_color(0.0f, 0.0f, 1.0f);
  }

  void set_status_led(esphome::light::LightState *led) { this->status_led_ = led; }

  void set_custom_base_topic(const std::string &topic) { this->custom_base_topic_ = topic; }

  void add_target_meter_id(uint32_t meter_id) {
    this->filter_meter_ids_.push_back(meter_id);
  }

  template<typename F>
  void add_on_json_callback(F &&callback) {
    this->json_callback_.add(std::forward<F>(callback));
  }

  void set_led_color(float r_coeff, float g_coeff, float b_coeff) {
    this->current_r_ = r_coeff;
    this->current_g_ = g_coeff;
    this->current_b_ = b_coeff;

    if (this->status_led_ == nullptr) return;

#ifdef USE_LIGHT
    auto call = this->status_led_->make_call();
    call.set_state(true);
    call.set_brightness(this->brightness_scale_);
    call.set_rgb(this->current_r_, this->current_g_, this->current_b_);
    call.perform();
#endif
  }

  enum class DecodeResult {
    TOO_SHORT = 0,
    UNSUPPORTED_PROTOCOL,
    SUCCESS,
    CRC_FAILED
  };

  bool on_receive(esphome::remote_base::RemoteReceiveData data) override {
    this->receive_call_count_++;

    this->cancel_timeout("scm_inactivity_watchdog");
    this->set_timeout("scm_inactivity_watchdog", 600000, [this]() {
      if (this->receive_call_count_ == 0) {
        this->set_led_color(1.0f, 0.0f, 1.0f);
      } else {
        this->set_led_color(1.0f, 1.0f, 0.0f);
      }
      ESP_LOGD("scm_status", "No data received for 10 minutes (%u calls).", static_cast<unsigned int>(this->receive_call_count_));
    });

    std::vector<bool> raw_bits = remote_receiver_helpers::extract_raw_bits(data);

    if (raw_bits.size() < 100) {
      ESP_LOGD("scm_decode", "Received transmission ignored: too short (%zu bits)", raw_bits.size());
      return false; 
    }

    std::vector<bool> even_stream;
    even_stream.reserve(raw_bits.size() / 2);
    for (size_t i = 0; i < raw_bits.size(); i += 2) {
      even_stream.push_back(raw_bits[i]);
    }

    std::vector<bool> odd_stream;
    odd_stream.reserve(raw_bits.size() / 2);
    for (size_t i = 1; i < raw_bits.size(); i += 2) {
      odd_stream.push_back(raw_bits[i]);
    }

    DecodeResult decode_result = DecodeResult::UNSUPPORTED_PROTOCOL;
    uint8_t protocol_id = 0;
    uint8_t meter_type = 0;
    uint32_t meter_id = 0;
    uint32_t consumption = 0;
    uint16_t tamper = 0;
    uint16_t expected_crc = 0;
    uint16_t calculated_crc = 0;
    std::optional<size_t> sync_idx = std::nullopt;
    uint8_t sync_bit_errors = 0;
    int matched_variation = 0;
    std::vector<bool> *active_target_stream = nullptr;

    // Notice std::vector<bool> &stream is now non-const to allow correction mutation
    auto decode_stream = [&](std::vector<bool> &stream, size_t s_idx, uint8_t &p_id, uint8_t &m_type, uint32_t &m_id, uint32_t &cons, uint16_t &tamp, uint16_t &exp_crc, uint16_t &calc_crc) -> DecodeResult {
      size_t payload_idx = s_idx + 12;
      if (stream.size() < payload_idx + 96 + 16) return DecodeResult::TOO_SHORT;

      p_id = bit_stream_handling::read_bits(stream, payload_idx + 0, 8);
      if (p_id != 0x1e) return DecodeResult::UNSUPPORTED_PROTOCOL;

      m_type = bit_stream_handling::read_bits(stream, payload_idx + 8, 8) & 0x0F;
      m_id = bit_stream_handling::read_bits(stream, payload_idx + 16, 32);
      cons = bit_stream_handling::read_bits(stream, payload_idx + 48, 32);
      tamp = bit_stream_handling::read_bits(stream, payload_idx + 80, 16);
      exp_crc = bit_stream_handling::read_bits(stream, payload_idx + 96, 16);

      calc_crc = bit_stream_handling::calculate_crc16_ccitt(stream, payload_idx, 96);

      if (calc_crc != exp_crc) {
        ESP_LOGW("scm_checksum", "Checksum mismatch: calculated 0x%04X, expected 0x%04X", calc_crc, exp_crc);
        return DecodeResult::CRC_FAILED;
      }

      // Can cause corruption if more than single bit errors present
      // if (calc_crc != exp_crc) {
      //   // Attempt O(1) single-bit error correction via syndrome lookup table
      //   if (bit_stream_handling::correct_single_bit_error_lookup(stream, payload_idx, 96, exp_crc, calc_crc)) {
      //     ESP_LOGI("scm_checksum", "Successfully corrected single-bit error via syndrome lookup!");
      //     calc_crc = exp_crc; // Re-sync checksum validation state after fix
      //   } else {
      //     ESP_LOGW("scm_checksum", "Checksum mismatch: calculated 0x%04X, expected 0x%04X", calc_crc, exp_crc);
      //     return DecodeResult::CRC_FAILED;
      //   }
      // }

      return DecodeResult::SUCCESS;
    };

    // Loop through stream variations using an integer (0: even, 1: odd, 2: inverted even, 3: inverted odd)
    for (int variation = 0; variation < 4; variation++) {
      bool is_odd = (variation == 1 || variation == 3);
      bool is_inverted = (variation == 2 || variation == 3);

      std::vector<bool> *target_stream = nullptr;
      if (!is_inverted) {
        target_stream = is_odd ? &odd_stream : &even_stream;
      } else {
        if (is_odd) {
          odd_stream.flip();
          target_stream = &odd_stream;
        } else {
          even_stream.flip();
          target_stream = &even_stream;
        }
      }

      auto result = bit_stream_handling::scan_stream(*target_stream, 0x6A3, 12, 96, 1);
      if (result.has_value()) {
        sync_idx = result;
        sync_bit_errors = __builtin_popcount(bit_stream_handling::read_bits(*target_stream, sync_idx.value(), 12) ^ 0x6A3);
        matched_variation = variation;
        active_target_stream = target_stream;
        ESP_LOGD("scm_decode", "SCM+ Sync found at index: %zu with variation: %d", sync_idx.value(), variation);
        break;
      }
    }

    if (!sync_idx.has_value()) {
      ESP_LOGD("scm_decode", "Sync was not found");
      return false;
    }

    decode_result = decode_stream(*active_target_stream, sync_idx.value(), protocol_id, meter_type, meter_id, consumption, tamper, expected_crc, calculated_crc);

    if (decode_result == DecodeResult::TOO_SHORT) {
      ESP_LOGD("scm_decode", "Not enough bits received (%zu bits)", active_target_stream->size());
      return false;
    }

    if (decode_result == DecodeResult::UNSUPPORTED_PROTOCOL) {
      ESP_LOGD("scm_decode", "Unsupported protocol ID (0x%02X)", static_cast<unsigned int>(protocol_id));
      return false;
    }

    ESP_LOGI("scm_decode", "SCM+ Packet Decoded (Sync Errors: %u) -> Protocol ID: 0x%02X | Meter Type: 0x%02X | Meter ID: %u | Consumption: %u | Tamper: 0x%04X | CRC Expected: 0x%04X | CRC Calculated: 0x%04X",
             static_cast<unsigned int>(sync_bit_errors),
             static_cast<unsigned int>(protocol_id),
             static_cast<unsigned int>(meter_type),
             static_cast<unsigned int>(meter_id),
             static_cast<unsigned int>(consumption),
             static_cast<unsigned int>(tamper),
             static_cast<unsigned int>(expected_crc),
             static_cast<unsigned int>(calculated_crc));

    if (!this->filter_meter_ids_.empty()) {
      auto it = std::find(this->filter_meter_ids_.begin(), this->filter_meter_ids_.end(), meter_id);
      if (it == this->filter_meter_ids_.end()) {
        std::string allowed_str = "";
        for (size_t k = 0; k < this->filter_meter_ids_.size(); k++) {
          allowed_str += std::to_string(this->filter_meter_ids_[k]);
          if (k + 1 < this->filter_meter_ids_.size()) {
            allowed_str += ", ";
          }
        }
        ESP_LOGI("scm_filter", "Meter ID %u ignored (not in allowed list: [%s])",
                 static_cast<unsigned int>(meter_id), allowed_str.c_str());
        return false; 
      }
    }

    if (decode_result == DecodeResult::CRC_FAILED) {
      ESP_LOGD("scm_status", "SCM+ packet CRC validation failed");
      this->set_led_color(1.0f, 0.0f, 0.0f);
      return false;
    }

    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["meter_id"] = meter_id;
    root["consumption"] = consumption;
    root["meter_type"] = meter_type;
    root["tamper"] = tamper;
    root["protocol_id"] = protocol_id;
    this->json_callback_.call(root);

#ifdef USE_MQTT
    if (esphome::mqtt::global_mqtt_client != nullptr && esphome::mqtt::global_mqtt_client->is_connected()) {
      std::string base_topic = this->custom_base_topic_;
      if (base_topic.empty()) {
        base_topic = "cc1101/" + esphome::App.get_name();
      }

      std::string mqtt_topic = base_topic + "/" + std::to_string(meter_id);
      std::string mqtt_payload = std::to_string(consumption);
      
      esphome::mqtt::global_mqtt_client->publish(mqtt_topic, mqtt_payload, 0, true);
      ESP_LOGI("scm_mqtt", "MQTT Stream Success -> Published to %s: %s", mqtt_topic.c_str(), mqtt_payload.c_str());
      
      ESP_LOGD("scm_status", "Successful MQTT publish / valid packet processed");
      this->set_led_color(0.0f, 1.0f, 0.0f);
      return true;
    } else {
      ESP_LOGW("scm_mqtt", "MQTT Stream Skipped -> Client disconnected from broker.");
      ESP_LOGD("scm_status", "MQTT client disconnected from broker");
      this->set_led_color(1.0f, 0.0f, 0.0f);
      return false;
    }
#else
    ESP_LOGD("scm_status", "Valid packet processed (MQTT disabled)");
    this->set_led_color(0.0f, 1.0f, 0.0f);
    return true;
#endif
  }
};

}  // namespace scm_plus
