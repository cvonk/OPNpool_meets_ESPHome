#include "opnpool.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opnpool {

static const char *const TAG = "opnpool";

climate::ClimateTraits OpnPoolClimate::traits() {
  auto traits = climate::ClimateTraits();
  
  // 1. Current Temperature is a standard feature flag
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  
  // 2. Target Temperature is enabled by adding ACTION or specifically 
  // defining the visual range. In many versions, it's actually 
  // CLIMATE_SUPPORTS_ACTION or simply enabled by default when 
  // min/max temperatures are set.
  
  // 3. Define the visual range (this often implicitly enables the target slider)
  traits.set_visual_min_temperature(18);
  traits.set_visual_max_temperature(35);
  traits.set_visual_temperature_step(1);
  
  // 4. Custom presets
  traits.set_supported_custom_presets({"None", "Heat", "Solar Preferred", "Solar"});
  
  return traits;
}

void OpnPoolClimate::control(const climate::ClimateCall &call) {

  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    // Example: Send target temp as a 1-byte payload
    uint8_t target = (uint8_t)this->target_temperature;
    this->parent_->write_packet(0x02, {target}); 
  }

  if (call.get_custom_preset() != nullptr) {
    const char* preset_ptr = call.get_custom_preset();
    std::string preset_str = preset_ptr; 
    this->set_custom_preset_(preset_ptr);

    if (preset_str == "Heat") {
      this->parent_->write_packet(0x03, {0x01}); // Command 0x03, Data 0x01 (On)
    } else if (preset_str == "None") {
      this->parent_->write_packet(0x03, {0x00}); // Command 0x03, Data 0x00 (Off)
    }
  }
  this->publish_state();
}


void OpnPool::setup() {
  // No manual serial initialization needed; UARTDevice handles it.
}

void OpnPool::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);

    // Header Sync (0xFF 0xAA)
    if (this->rx_buffer_.size() > 0 && this->rx_buffer_[0] != 0xFF) {
      this->rx_buffer_.clear();
      continue;
    }
    if (this->rx_buffer_.size() > 1 && this->rx_buffer_[1] != 0xAA) {
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }

    if (this->rx_buffer_.size() < 4) continue;
    uint8_t len = this->rx_buffer_[2];

    if (this->rx_buffer_.size() < (size_t)(len + 6)) continue;

    this->parse_packet_(this->rx_buffer_);
    this->rx_buffer_.clear();
  }
}

void OpnPool::write_packet(uint8_t command, const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> pkt;
  pkt.push_back(0xFF); pkt.push_back(0xAA);
  pkt.push_back((uint8_t)payload.size());
  pkt.push_back(command);
  for (uint8_t b : payload) pkt.push_back(b);

  uint16_t crc = pkt[2] + pkt[3];
  for (uint8_t b : payload) crc += b;
  pkt.push_back((uint8_t)(crc >> 8));
  pkt.push_back((uint8_t)(crc & 0xFF));

  // This automatically handles the RS485 Direction/RE/DE pin
  this->write_array(pkt);
}

void OpnPool::dump_config() {
  ESP_LOGCONFIG(TAG, "OpnPool:");
  
  // Log Climate Entities
  LOG_CLIMATE("  ", "Pool Heater", this->pool_heater_);
  LOG_CLIMATE("  ", "Spa Heater", this->spa_heater_);

  // Log Sensors
  LOG_SENSOR("  ", "Water Temperature", this->water_temp_s_);
  LOG_SENSOR("  ", "Air Temperature", this->air_temp_s_);
  LOG_SENSOR("  ", "Pump Power", this->pump_power_s_);

  // Log Switches
  LOG_SWITCH("  ", "Pool Switch", this->pool_sw_);
  LOG_SWITCH("  ", "Spa Switch", this->spa_sw_);

  // Log Binary Sensors
  LOG_BINARY_SENSOR("  ", "Pump Running", this->pump_running_bs_);
  
  // Log Text Sensors
  LOG_TEXT_SENSOR("  ", "System Time", this->system_time_ts_);
  ESP_LOGCONFIG(TAG, "OpnPool (UART Device)");
  this->check_uart_settings(9600); // Verify baud rate in logs
}

void OpnPool::parse_packet_(const std::vector<uint8_t> &data) {
  uint8_t cmd = data[3];

  // Assuming Command 0x01 is the status update containing temperature at index 5
  if (cmd == 0x01 && data.size() > 5) {
    float current_temp = data[5];

    // 1. Update the standalone Water Temperature sensor
    if (this->water_temp_s_ != nullptr) {
      this->water_temp_s_->publish_state(current_temp);
    }

    // 2. Feed temperature into Pool Heater climate
    if (this->pool_heater_ != nullptr) {
      this->pool_heater_->current_temperature = current_temp;
      this->pool_heater_->publish_state();
    }

    // 3. Feed temperature into Spa Heater climate
    if (this->spa_heater_ != nullptr) {
      this->spa_heater_->current_temperature = current_temp;
      this->spa_heater_->publish_state();
    }
    
    ESP_LOGD(TAG, "Updated temperature to: %.1f°C", current_temp);
  }
}

} // namespace opnpool
} // namespace esphome