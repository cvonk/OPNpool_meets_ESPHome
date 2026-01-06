#include "opnpool.h"
#include "esphome/core/log.h"
#include <time.h>
#include <esp_system.h>
#include <cstdlib>

#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "poolstate.h"
#include "ipc.h"
#include "pool_task.h"

namespace esphome {
namespace opnpool {
  
// get base file name at compiler time
constexpr const char* base_file(const char* path) {
    const char* file = path;
    while (*path) {
        if (*path++ == '/') {
            file = path;
        }
    }
    return file;
}

static const char *const TAG = "opnpool";

climate::ClimateTraits OpnPoolClimate::traits() {
    auto traits = climate::ClimateTraits();
    
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);  
    traits.set_visual_min_temperature(32);
    traits.set_visual_max_temperature(110);
    traits.set_visual_temperature_step(1);
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

#if 0
void
test_task(void * ipc_void)
{
    ESP_LOGI(TAG, "test_task initializing ..");

    pool_task(ipc_void);

    while (1) {
        ESP_LOGI(TAG, "test_task heartbeat ..");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
#endif

void OpnPool::setup() {

    ESP_LOGI(TAG, "setup ..");  // only viewable on the serial console (WiFi not yet started)

    this->ipc_.to_pool_q = xQueueCreate(10, sizeof(ipc_to_pool_msg_t));
    this->ipc_.to_mqtt_q = xQueueCreate(60, sizeof(ipc_to_mqtt_msg_t));
    assert(this->ipc_.to_mqtt_q && this->ipc_.to_pool_q);

    // spin off a pool_task that handles RS485 and the pool state machine
    //BaseType_t const res = xTaskCreate(&pool_task, "pool_task", 2*4096, &this->ipc_, 5, NULL);
    BaseType_t const res = xTaskCreate(&pool_task, "pool_task", 2*4096, &this->ipc_, 3, NULL);
    if (res != pdPASS) {
      ESP_LOGE(TAG, "Failed to create pool_task! Error code: %d", res);
    } else {
      ESP_LOGI(TAG, "pool_task created successfully.");
    } 
}

void OpnPool::loop() {

    // do whatever mqtt_task did in the past

    ipc_to_mqtt_msg_t msg;

    if (xQueueReceive(this->ipc_.to_mqtt_q, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

        ESP_LOGV(TAG, "Received message from pool_task (%u)", msg.dataType);
        switch (msg.dataType) {

            // publish using topic and message from `msg.data`
            case IPC_TO_MQTT_TYP_PUBLISH: {  // publish a message (from `hass_task`)

                ESP_LOGV(TAG, "Publishing MQTT message: %s", msg.data);

                char const * const topic = msg.data;
                char * message = strchr(msg.data, '\t');
                *message++ = '\0';
                ESP_LOGV(TAG, "tx %s: %s", topic, message);

                //esp_mqtt_client_publish(client, topic, message, strlen(message), 1, 0);
                free(msg.data);
                break;
            }
        }
    }

#if 0
  while (this->available()) {    
    uint8_t byte;
    this->read_byte(&byte);
    
    ESP_LOGV(TAG, "Rx: %02X", byte);

    this->rx_buffer_.push_back(byte);

    // header Sync (0xFF 0xAA)
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

    //if (this->air_temperature_sensor_ != nullptr) {
    //  this->air_temperature_sensor_->publish_state(new_temp_value);
    //}
  }
#endif
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

void OpnPool::parse_packet_(const std::vector<uint8_t> &data) {

#ifdef NOT_YET
    if (should_log_(datalink_level_, ESPHOME_LOG_LEVEL_VERBOSE)) {
          ESP_LOGV("datalink", "Raw packet received: %s", format_hex_pretty(data).c_str());
      }

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
#endif
}

void OpnPool::dump_config() {

    ESP_LOGCONFIG(TAG, "OpnPool:");

#if 0

    // climate entities
    LOG_CLIMATE("  ", "Pool Heater", this->pool_heater_);
    LOG_CLIMATE("  ", "Spa Heater", this->spa_heater_);

    // switches
    LOG_SWITCH("  ", "Pool switch", this->pool_sw_);
    LOG_SWITCH("  ", "Spa switch", this->spa_sw_);
    LOG_SWITCH("  ", "Aux1 switch", this->aux1_sw_);
    LOG_SWITCH("  ", "Aux2 switch", this->aux2_sw_);
    LOG_SWITCH("  ", "Feature1 switch", this->feature1_sw_);
    LOG_SWITCH("  ", "Feature2 switch", this->feature2_sw_);
    LOG_SWITCH("  ", "Feature3 switch", this->feature3_sw_);
    LOG_SWITCH("  ", "Feature4 switch", this->feature4_sw_);

    // sensors
    LOG_SENSOR("  ", "Water temperature", this->water_temp_s_);
    LOG_SENSOR("  ", "Air temperature", this->air_temp_s_);
    LOG_SENSOR("  ", "Pump power", this->pump_power_s_);
    LOG_SENSOR("  ", "Pump flow", this->pump_flow_s_);
    LOG_SENSOR("  ", "Pump speed", this->pump_speed_s_);
    LOG_SENSOR("  ", "Pump status", this->pump_status_s_);
    LOG_SENSOR("  ", "Pump state", this->pump_state_s_);
    LOG_SENSOR("  ", "Pump error", this->pump_error_s_);
    LOG_SENSOR("  ", "Chlorinator level", this->chlor_level_s_);
    LOG_SENSOR("  ", "Chlorinator salt", this->chlor_salt_s_);
    
    // binary sensors
    LOG_BINARY_SENSOR("  ", "Pump running", this->pump_running_bs_);
    LOG_BINARY_SENSOR("  ", "Mode service", this->mode_service_bs_);
    LOG_BINARY_SENSOR("  ", "Mode temperature increase", this->mode_temp_inc_bs_);
    LOG_BINARY_SENSOR("  ", "Mode freeze protection", this->mode_freeze_bs_);
    LOG_BINARY_SENSOR("  ", "Mode timeout", this->mode_timeout_bs_);

    // text sensors
    LOG_TEXT_SENSOR("  ", "Pool schedule", this->pool_sched_ts_);
    LOG_TEXT_SENSOR("  ", "Spa schedule", this->spa_sched_ts_);
    LOG_TEXT_SENSOR("  ", "AUX1 schedule", this->aux1_sched_ts_);
    LOG_TEXT_SENSOR("  ", "AUX2 schedule", this->aux2_sched_ts_);
    LOG_TEXT_SENSOR("  ", "Pump mode", this->pump_mode_ts_);
    LOG_TEXT_SENSOR("  ", "Chlorinator name", this->chlor_name_ts_);
    LOG_TEXT_SENSOR("  ", "Chlorinator status", this->chlor_status_ts_);
    LOG_TEXT_SENSOR("  ", "System time", this->system_time_ts_);
    LOG_TEXT_SENSOR("  ", "Controller f/w version", this->controller_fw_ts_);
    LOG_TEXT_SENSOR("  ", "Interface f/w version", this->interface_fw_ts_);

#if 0
    // log levels
    auto log_level_to_str = [](int level) -> const char* {
        switch (level) {
            case ESPHOME_LOG_LEVEL_NONE:         return "NONE";
            case ESPHOME_LOG_LEVEL_ERROR:        return "ERROR";
            case ESPHOME_LOG_LEVEL_WARN:         return "WARN";
            case ESPHOME_LOG_LEVEL_INFO:         return "INFO";
            case ESPHOME_LOG_LEVEL_CONFIG:       return "CONFIG";
            case ESPHOME_LOG_LEVEL_DEBUG:        return "DEBUG";
            case ESPHOME_LOG_LEVEL_VERBOSE:      return "VERBOSE";
            case ESPHOME_LOG_LEVEL_VERY_VERBOSE: return "VERY_VERBOSE";
            default:                     return "UNKNOWN";
        }
    };
#endif

#endif    
}

} // namespace opnpool
} // namespace esphome