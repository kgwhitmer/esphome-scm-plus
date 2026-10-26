#pragma once

#include "esphome.h"
#include "esphome/core/application.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/mqtt/mqtt_client.h" 
#include <cmath> 
#include <vector>
#include <string>
#include <algorithm>

#include "remote_receiver_helpers.h"
#include "bit_stream_handling.h"

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

  ScmPlusDecoder(float brightness) : brightness_scale_(brightness) {}

  void setup() override {
    if (this->status_led_ != nullptr) {
      ESP_LOGD("scm_led", "LED: Blue - Component initialization / boot");
      this->set_led_color(0.0f, 0.0f, 1.0f);  // Solid Blue
    }
  }

  void set_status_led(esphome::light::LightState *led) { this->status_led_ = led; }

  void set_custom_base_topic(const std::string &topic) { this->custom_base_topic_ = topic; }

  void add_target_meter_id(uint32_t meter_id) {
    this->filter_meter_ids_.push_back(meter_id);
  }

  void set_led_color(float r_coeff, float g_coeff, float b_coeff) {
    this->current_r_ = r_coeff;
    this->current_g_ = g_coeff;
    this->current_b_ = b_coeff;

    if (this->status_led_ == nullptr) return;

    auto call = this->status_led_->make_call();
    call.set_state(true);
    call.set_brightness(this->brightness_scale_);
    call.set_rgb(this->current_r_, this->current_g_, this->current_b_);
    call.perform();
  }

  enum class DecodeResult {
    NOT_SCM = 0,
    SUCCESS,
    CRC_FAILED
  };

  bool on_receive(esphome::remote_base::RemoteReceiveData data) override {
    this->receive_call_count_++;

    this->cancel_timeout("scm_inactivity_watchdog");
    this->set_timeout("scm_inactivity_watchdog", 600000, [this]() {
      if (this->receive_call_count_ == 0) {
        this->set_led_color(1.0f, 0.0f, 1.0f);  // Solid Purple
      } else {
        this->set_led_color(1.0f, 1.0f, 0.0f);  // Solid Yellow
      }
      ESP_LOGD("scm_status", "No data received for 10 minutes (%u calls).", static_cast<unsigned int>(this->receive_call_count_));
    });

    std::vector<bool> raw_bits = remote_receiver_helpers::extract_raw_bits(data);

    if (raw_bits.size() < 100) {
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

    DecodeResult decode_result = DecodeResult::NOT_SCM;
    uint8_t protocol_id = 0;
    uint8_t meter_type = 0;
    uint32_t meter_id = 0;
    uint32_t consumption = 0;
    uint16_t tamper = 0;
    uint16_t expected_crc = 0;
    uint16_t calculated_crc = 0;

    auto decode_stream = [&](const std::vector<bool> &stream, size_t sync_idx, uint8_t &p_id, uint8_t &m_type, uint32_t &m_id, uint32_t &cons, uint16_t &tamp, uint16_t &exp_crc, uint16_t &calc_crc) -> DecodeResult {
      size_t payload_idx = sync_idx + 12;
      if (stream.size() < payload_idx + 96 + 16) return DecodeResult::NOT_SCM;

      p_id = bit_stream_handling::read_bits(stream, payload_idx + 0, 8);
      if (p_id != 0x1e) return DecodeResult::NOT_SCM;

      // Extract all payload fields regardless of CRC so we can inspect/log them on failure
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

      return DecodeResult::SUCCESS;
    };

    for (int i = 0; i < 2; i++) {
      const auto &primary = (i == 0) ? even_stream : odd_stream;
      const auto &secondary = (i == 0) ? odd_stream : even_stream;

      auto result = bit_stream_handling::scan_stream(primary, 0x6A3, 12, 96);
      if (!result.has_value()) {
        continue;
      }

      decode_result = decode_stream(primary, result.value(), protocol_id, meter_type, meter_id, consumption, tamper, expected_crc, calculated_crc);

      if (decode_result == DecodeResult::SUCCESS) {
        break;
      }

      if (decode_result == DecodeResult::NOT_SCM) {
        return false;
      }

      // CRC error -> invert the unused stream and try
      std::vector<bool> inverted_sec = secondary;
      for (size_t k = 0; k < inverted_sec.size(); k++) {
        inverted_sec[k] = !inverted_sec[k];
      }

      auto inv_result = bit_stream_handling::scan_stream(inverted_sec, 0x6A3, 12, 96);
      if (inv_result.has_value()) {
        DecodeResult inv_res = decode_stream(inverted_sec, inv_result.value(), protocol_id, meter_type, meter_id, consumption, tamper, expected_crc, calculated_crc);
        if (inv_res == DecodeResult::SUCCESS) {
          decode_result = DecodeResult::SUCCESS;
        } else if (inv_res == DecodeResult::CRC_FAILED) {
          decode_result = DecodeResult::CRC_FAILED;
        }
      }

      break;
    }

    if (decode_result == DecodeResult::NOT_SCM) {
      return false;
    }

    // If a target meter ID filter list is configured, drop packets from any meter not present in the list
    if (!this->filter_meter_ids_.empty()) {
      auto it = std::find(this->filter_meter_ids_.begin(), this->filter_meter_ids_.end(), meter_id);
      if (it == this->filter_meter_ids_.end()) {
        return false; 
      }
    }

    // Log all decoded packet field values before validating the CRC
    ESP_LOGI("scm_decode", "SCM+ Packet Decoded -> Protocol ID: 0x%02X | Meter Type: 0x%02X | Meter ID: %u | Consumption: %u | Tamper: 0x%04X | CRC Expected: 0x%04X | CRC Calculated: 0x%04X",
             static_cast<unsigned int>(protocol_id),
             static_cast<unsigned int>(meter_type),
             static_cast<unsigned int>(meter_id),
             static_cast<unsigned int>(consumption),
             static_cast<unsigned int>(tamper),
             static_cast<unsigned int>(expected_crc),
             static_cast<unsigned int>(calculated_crc));

    // Evaluate CRC validity and handle red LED feedback if check fails
    if (decode_result == DecodeResult::CRC_FAILED) {
      ESP_LOGD("scm_led", "LED: Red - SCM+ packet CRC validation failed");
      this->set_led_color(1.0f, 0.0f, 0.0f);  // Solid Red
      return false;
    }

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
      
      ESP_LOGD("scm_led", "LED: Green - Successful MQTT publish / valid packet processed");
      this->set_led_color(0.0f, 1.0f, 0.0f);  // Solid Green
      return true;
    } else {
      ESP_LOGW("scm_mqtt", "MQTT Stream Skipped -> Client disconnected from broker.");
      ESP_LOGD("scm_led", "LED: Red - MQTT client disconnected from broker");
      this->set_led_color(1.0f, 0.0f, 0.0f);  // Solid Red
      return false;
    }
#else
    ESP_LOGD("scm_led", "LED: Green - Valid packet processed (MQTT disabled)");
    this->set_led_color(0.0f, 1.0f, 0.0f);  // Solid Green
    return true;
#endif
  }
};

}  // namespace scm_plus
