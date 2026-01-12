#include <time.h>
#include <esp_system.h>
#include <cstdlib>
#include <esphome/core/log.h>

#include "opnpool.h"
#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "opnpoolstate.h"
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

    uint8_t const idx = this->parent_->get_climate_index(this);

    uint8_t const pool_idx = static_cast<uint8_t>(network_pool_circuit_t::POOL);
    uint8_t const spa_idx = static_cast<uint8_t>(network_pool_circuit_t::SPA);

       // get thermostat settings (we need the current settings, as we may only change one of them)

    poolstate_thermo_t thermos_old[POOLSTATE_THERMO_TYP_COUNT];
    poolstate_thermo_t thermos_new[POOLSTATE_THERMO_TYP_COUNT];
    {
        poolstate_t state;
        parent_->get_opnpool_state()->get(&state);
        memcpy(thermos_old, state.thermos, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT);
        memcpy(thermos_new, state.thermos, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT);
    }

        // handle mode changes (OFF, HEAT, AUTO)

    if (call.get_mode().has_value()) {
        climate::ClimateMode mode = *call.get_mode();
        
        ESP_LOGV(TAG, "Climate mode changed: idx=%u to mode=%d", idx, static_cast<int>(mode));
        
            // map climate index to circuit index
        uint8_t circuit_idx = idx == pool_idx ? static_cast<uint8_t>(network_pool_circuit_t::POOL) : static_cast<uint8_t>(network_pool_circuit_t::SPA);
        
        switch (mode) {
            case climate::CLIMATE_MODE_OFF:
                this->parent_->on_switch_command(circuit_idx, false);
                break;
                
            case climate::CLIMATE_MODE_HEAT:
            case climate::CLIMATE_MODE_AUTO:
                this->parent_->on_switch_command(circuit_idx, true);
                break;
                
            default:
                ESP_LOGW(TAG, "Unsupported climate mode: %d", static_cast<int>(mode));
                break;
        }
    }

        // handle target temperature changes

    if (call.get_target_temperature().has_value()) {
        float target_temp_celsius = *call.get_target_temperature();
        
            // convert to Fahrenheit (ESPHome uses Celsius internally)
        float target_temp_fahrenheit = target_temp_celsius * 9.0f / 5.0f + 32.0f;
        
        ESP_LOGV(TAG, "Target temperature changed: idx=%u to %.1f°F", idx, target_temp_fahrenheit);

        thermos_new[idx].set_point = static_cast<uint8_t>(target_temp_fahrenheit);
    }

        // handle heat source changes

    const char* preset_str = call.get_custom_preset();
    if (preset_str != nullptr) {

        ESP_LOGV(TAG, "Heat source changed: idx=%u to %s", idx, preset_str);

        if (strcasecmp(preset_str, "None") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::NONE);
        } else if (strcasecmp(preset_str, "Heat") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::HEATER);
        } else if (strcasecmp(preset_str, "Solar Preferred") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR_PREF);
        } else if (strcasecmp(preset_str, "Solar") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR);
        }
    }

    uint8_t const pool_heat_src = thermos_new[pool_idx].heat_src;
    uint8_t const spa_heat_src = thermos_new[spa_idx].heat_src;
    bool const thermos_changed = memcmp(thermos_old, thermos_new, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT) != 0;

        // actuate thermostat changes if necessary

    if (thermos_changed) {

        network_msg_t msg = {
            .typ = network_msg_typ_t::CTRL_HEAT_SET,
            .u = {
                .ctrl_heat_set = {
                    .poolSetpoint = thermos_new[pool_idx].set_point,
                    .spaSetpoint = thermos_new[spa_idx].set_point,
                    .heatSrc = static_cast<uint8_t>(pool_heat_src | (spa_heat_src << 2))
                },
            },
        };

        ESP_LOGV(TAG, "Sending HEAT_SET: pool_SP=%u°F, spa_SP=%u°F, heat_src=0x%02X", 
                  msg.u.ctrl_heat_set.poolSetpoint, 
                  msg.u.ctrl_heat_set.spaSetpoint,
                  msg.u.ctrl_heat_set.heatSrc);
        ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc());
    }
    
        // DON'T publish state here - wait for pool controller confirmation
        // State will be published by update_climates() when pool responds
}

void OpnPoolSwitch::write_state(bool state) {

  if (this->parent_) {
    
        // send command but DON'T publish yet
    this->parent_->on_switch_command(this->circuit_id_, state);
    
       // store as pending
    this->parent_->add_pending_switch(this, state);
  }
}

void OpnPool::on_switch_command(uint8_t const circuit_id, bool const state) {

    network_msg_t msg = {
        .typ = network_msg_typ_t::CTRL_CIRCUIT_SET,
        .u = {
            .ctrl_circuit_set = {
              .circuit = static_cast<uint8_t>(circuit_id + 1),
              .value = state ? (uint8_t)1 : (uint8_t)0,          
            },
        },
    };

    ESP_LOGVV(TAG, "Sending CIRCUIT_SET command: circuit=%u to %u", msg.u.ctrl_circuit_set.circuit, msg.u.ctrl_circuit_set.value);
    ipc_send_network_msg_to_pool_task(&msg, &this->ipc_);
}

OpnPool::OpnPool() {
    opnPoolState_ = new OpnPoolState(this);
}

void OpnPool::setup() {

    ESP_LOGV(TAG, "setup ..");  // only viewable on the serial console (WiFi not yet started)

    if (!opnPoolState_->is_valid()) {
        ESP_LOGE(TAG, "Failed to initialize opnPoolState_");
        return;
    }

    this->ipc_.to_pool_q = xQueueCreate(6, sizeof(network_msg_t));
    this->ipc_.to_main_q = xQueueCreate(10, sizeof(network_msg_t));
    assert(this->ipc_.to_main_q && this->ipc_.to_pool_q);

    // spin off a pool_task that handles RS485 and the pool state machine
    BaseType_t const res = xTaskCreate(&pool_task, "pool_task", 2*4096, &this->ipc_, 3, NULL);
    if (res != pdPASS) {
      ESP_LOGE(TAG, "Failed to create pool_task! Error code: %d", res);
    } else {
      ESP_LOGI(TAG, "pool_task created successfully.");
    } 
}

void
OpnPool::service_requests_from_pool(ipc_t const * const ipc)
{
    network_msg_t msg;

    if (xQueueReceive(ipc->to_main_q, &msg, 0) == pdPASS) {  // don't block, just check if a message is available

        //ESP_LOGV(TAG, "Handling msg typ=%s", ipc_to_home_typ_str(queued_msg.typ));

        if (opnPoolState_->rx_update(&msg) == ESP_OK) {

            ESP_LOGVV(TAG, "Poolstate changed");

            // 2BD: publish this as an update to the HA sensors 
            // Maybe2, that is where the OpnPoolSwitch, OpnPoolClimate et al classes are for.
        }
    }
}

void OpnPool::loop() {  //  this will be called repeatedly by the main ESPHome loop

    // don't do any blocking operations here
    // don't call vTaskDelay() or blocking queue operations

    this->service_requests_from_pool(&this->ipc_);

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
    
    // Log climates
    LOG_CLIMATE("  ", "Pool heater", this->heaters_[static_cast<uint8_t>(poolstate_thermo_typ_t::POOL)]);
    LOG_CLIMATE("  ", "Spa heater", this->heaters_[static_cast<uint8_t>(poolstate_thermo_typ_t::SPA)]);

    // switches - using array
    LOG_SWITCH("  ", "Pool switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::POOL)]);
    LOG_SWITCH("  ", "Spa switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::SPA)]);
    LOG_SWITCH("  ", "Aux1 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX1)]);
    LOG_SWITCH("  ", "Aux2 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX2)]);
    LOG_SWITCH("  ", "Aux3 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX3)]);
    LOG_SWITCH("  ", "Feature1 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE1)]);
    LOG_SWITCH("  ", "Feature2 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE2)]);
    LOG_SWITCH("  ", "Feature3 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE3)]);
    LOG_SWITCH("  ", "Feature4 switch", this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE4)]);

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
}

void OpnPool::add_pending_switch(OpnPoolSwitch *sw, bool target_state) {

        // remove any existing pending for this switch
    pending_switches_.erase(
        std::remove_if(pending_switches_.begin(), pending_switches_.end(),
        [sw](const pending_switch_t &p) { return p.sw == sw; }),
        pending_switches_.end()
    );

        // add new pending
    ESP_LOGVV(TAG, "Adding pending switch: circuit=%u, requested=%u", sw->get_circuit_id(), target_state);
    pending_switches_.push_back({
        .sw = sw,
        .target_state = target_state,
        .timestamp = millis()
    });
}

void OpnPool::check_pending_switches(const poolstate_t *new_state) {
    
    auto it = pending_switches_.begin();
    
    while (it != pending_switches_.end()) {
        
        uint8_t circuit_id = it->sw->get_circuit_id();
        bool actual_state = new_state->circuits.active[circuit_id];
        
        if (actual_state == it->target_state) {
            ESP_LOGVV(TAG, "Switch confirmed: circuit=%u, state=%d", circuit_id, actual_state);
            it->sw->publish_state(actual_state);
            it = pending_switches_.erase(it);
        } else if (millis() - it->timestamp > 5000) {  // 5 second timeout
            ESP_LOGW(TAG, "Switch confirmation timeout, circuit=%u", circuit_id);
            it->sw->publish_state(actual_state);  // Publish actual state
            it = pending_switches_.erase(it);
        } else {
            ++it;
        }
    }
}

void OpnPool::update_text_sensors(const poolstate_t *new_state) {
    
        // update schedule text sensors

    if (this->pool_sched_ts_ != nullptr) {
        char sched_str[64];
        uint8_t pool_idx = static_cast<uint8_t>(network_pool_circuit_t::POOL);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[pool_idx].start / 60, new_state->scheds[pool_idx].start % 60,
            new_state->scheds[pool_idx].stop / 60, new_state->scheds[pool_idx].stop % 60);
        this->pool_sched_ts_->publish_state(sched_str);
    }
    
    if (this->spa_sched_ts_ != nullptr) {
        char sched_str[64];
        uint8_t spa_idx = static_cast<uint8_t>(network_pool_circuit_t::SPA);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[spa_idx].start / 60, new_state->scheds[spa_idx].start % 60,
            new_state->scheds[spa_idx].stop / 60, new_state->scheds[spa_idx].stop % 60);
        this->spa_sched_ts_->publish_state(sched_str);
    }
    
    if (this->aux1_sched_ts_ != nullptr) {
        char sched_str[64];
        uint8_t aux1_idx = static_cast<uint8_t>(network_pool_circuit_t::AUX1);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[aux1_idx].start / 60, new_state->scheds[aux1_idx].start % 60,
            new_state->scheds[aux1_idx].stop / 60, new_state->scheds[aux1_idx].stop % 60);
        this->aux1_sched_ts_->publish_state(sched_str);
    }
    
    if (this->aux2_sched_ts_ != nullptr) {
        char sched_str[64];
        uint8_t aux2_idx = static_cast<uint8_t>(network_pool_circuit_t::AUX2);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[aux2_idx].start / 60, new_state->scheds[aux2_idx].start % 60,
            new_state->scheds[aux2_idx].stop / 60, new_state->scheds[aux2_idx].stop % 60);
        this->aux2_sched_ts_->publish_state(sched_str);
    }
    
        // update pump mode text sensor

    if (this->pump_mode_ts_ != nullptr) {
        this->pump_mode_ts_->publish_state(std::to_string(new_state->pump.mode));
    }

        // update chlorinator text sensors
    
    if (this->chlor_name_ts_ != nullptr) {
        this->chlor_name_ts_->publish_state(new_state->chlor.name);
    }
    
    if (this->chlor_status_ts_ != nullptr) {
        const char* status_str = "Unknown";
        switch (new_state->chlor.status) {
            case poolstate_chlor_status_t::OK: status_str = "OK"; break;
            case poolstate_chlor_status_t::LOW_FLOW: status_str = "Low Flow"; break;
            case poolstate_chlor_status_t::LOW_SALT: status_str = "Low Salt"; break;
            case poolstate_chlor_status_t::HIGH_SALT: status_str = "High Salt"; break;
            case poolstate_chlor_status_t::CLEAN_CELL: status_str = "Clean Cell"; break;
            case poolstate_chlor_status_t::COLD: status_str = "Cold"; break;
            case poolstate_chlor_status_t::OTHER: status_str = "Other"; break;
        }
        this->chlor_status_ts_->publish_state(status_str);
    }
    
        // update system time text sensor

    if (this->system_time_ts_ != nullptr) {
        char time_str[32];
        snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
            new_state->system.tod.date.year, new_state->system.tod.date.month, new_state->system.tod.date.day,
            new_state->system.tod.time.hour, new_state->system.tod.time.minute);
        this->system_time_ts_->publish_state(time_str);
    }
    
        // update firmware version text sensors

    if (this->controller_fw_ts_ != nullptr) {
        char fw_str[16];
        snprintf(fw_str, sizeof(fw_str), "%d.%d", new_state->system.version.major, new_state->system.version.minor);
        this->controller_fw_ts_->publish_state(fw_str);
    }
    
    if (this->interface_fw_ts_ != nullptr) {
        this->interface_fw_ts_->publish_state("N/A");
    }
}

void OpnPool::update_analog_sensors(const poolstate_t *new_state) {

    if (this->air_temp_s_ != nullptr) {
        float air_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)].temp;
        //float air_temp_c = (air_temp_f - 32.0f) * 5.0f / 9.0f;
        this->air_temp_s_->publish_state(air_temp_f);
    }
    
    if (this->water_temp_s_ != nullptr) {
        float water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
        //float water_temp_c = (water_temp_f - 32.0f) * 5.0f / 9.0f;
        this->water_temp_s_->publish_state(water_temp_f);
    }
    
    if (this->pump_power_s_ != nullptr) {
        this->pump_power_s_->publish_state(new_state->pump.power);
    }
    
    if (this->pump_flow_s_ != nullptr) {
        this->pump_flow_s_->publish_state(new_state->pump.flow);
    }
    
    if (this->pump_speed_s_ != nullptr) {
        this->pump_speed_s_->publish_state(new_state->pump.speed);
    }
    
    if (this->pump_state_s_ != nullptr) {
        this->pump_state_s_->publish_state(new_state->pump.state);
    }

    if (this->pump_error_s_ != nullptr) {
        this->pump_error_s_->publish_state(new_state->pump.error);
    }

    if (this->chlor_level_s_ != nullptr) {
        this->chlor_level_s_->publish_state(new_state->chlor.level);
    }

    if (this->chlor_salt_s_ != nullptr) {
        this->chlor_salt_s_->publish_state(new_state->chlor.salt);
    }
}

void OpnPool::update_binary_sensors(const poolstate_t *new_state) {

    if (this->pump_running_bs_ != nullptr) {
        this->pump_running_bs_->publish_state(new_state->pump.running);
    }
    
    if (this->mode_service_bs_ != nullptr) {
        this->mode_service_bs_->publish_state(new_state->modes.set[static_cast<uint8_t>(poolstate_elem_modes_typ_t::SERVICE)]);
    }
    
    if (this->mode_temp_inc_bs_ != nullptr) {
        this->mode_temp_inc_bs_->publish_state(new_state->modes.set[static_cast<uint8_t>(poolstate_elem_modes_typ_t::TEMP_INC)]);
    }
    
    if (this->mode_freeze_bs_ != nullptr) {
        this->mode_freeze_bs_->publish_state(new_state->modes.set[static_cast<uint8_t>(poolstate_elem_modes_typ_t::FREEZE_PROT)]);
    }
    
    if (this->mode_timeout_bs_ != nullptr) {
        this->mode_timeout_bs_->publish_state(new_state->modes.set[static_cast<uint8_t>(poolstate_elem_modes_typ_t::TIMEOUT)]);
    }
}

void OpnPool::update_climates(const poolstate_t *new_state) {
    
        // first check pending climate changes
    check_pending_climates(new_state);
    
    uint8_t const water_temp_in_fahrenheit = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
    float water_temp_in_celsius = (water_temp_in_fahrenheit - 32.0f) * 5.0f / 9.0f;

        // update all thermostats
    for (uint8_t idx = 0; idx < POOLSTATE_THERMO_TYP_COUNT; idx++) {
        
        OpnPoolClimate *heater = this->heaters_[idx];
        if (heater == nullptr) {
            continue;
        }
        
        poolstate_thermo_t const * const thermo = &new_state->thermos[idx];

            // update temperatures
        heater->current_temperature = water_temp_in_celsius;
        heater->target_temperature = (thermo->set_point - 32.0f) * 5.0f / 9.0f;
        
            // update mode based on {circuit state, heating status}
        if (new_state->circuits.active[idx]) {
            if (thermo->heating) {
                heater->mode = climate::CLIMATE_MODE_HEAT;
            } else {
                heater->mode = climate::CLIMATE_MODE_AUTO;
            }
        } else {
            heater->mode = climate::CLIMATE_MODE_OFF;
        }
        
            // update action
        if (thermo->heating) {
            heater->action = climate::CLIMATE_ACTION_HEATING;
        } else if (new_state->circuits.active[idx]) {
            heater->action = climate::CLIMATE_ACTION_IDLE;
        } else {
            heater->action = climate::CLIMATE_ACTION_OFF;
        }
        
        heater->publish_state();
    }
}

void OpnPool::add_pending_climate(OpnPoolClimate *climate, bool has_setpoint, float setpoint_celsius,
                                  bool has_heat_src, uint8_t heat_src) {
    
        // remove any existing pending for this climate
    pending_climates_.erase(
        std::remove_if(pending_climates_.begin(), pending_climates_.end(),
        [climate](const pending_climate_t &p) { return p.climate == climate; }),
        pending_climates_.end()
    );
    
        // add new pending
    uint8_t idx = get_climate_index(climate);
    ESP_LOGVV(TAG, "Adding pending climate: circuit=%u, has_setpoint=%d (%.1f°C), has_heat_src=%d (0x%02X)", 
              idx, has_setpoint, setpoint_celsius, has_heat_src, heat_src);
    
    pending_climates_.push_back({
        .climate = climate,
        .target_setpoint_celsius = setpoint_celsius,
        .target_heat_src = heat_src,
        .has_setpoint_change = has_setpoint,
        .has_heat_src_change = has_heat_src,
        .timestamp = millis()
    });
}

void OpnPool::check_pending_climates(const poolstate_t *new_state) {
    
    auto it = pending_climates_.begin();
    
    while (it != pending_climates_.end()) {
        
        uint8_t circuit_idx = get_climate_index(it->climate);
        poolstate_thermo_t const * const thermo = &new_state->thermos[circuit_idx];
        
        bool setpoint_matches = true;
        bool heat_src_matches = true;
        
        // Check setpoint if it was changed
        if (it->has_setpoint_change) {
            float actual_setpoint_fahrenheit = thermo->set_point;
            float target_setpoint_fahrenheit = it->target_setpoint_celsius * 9.0f / 5.0f + 32.0f;
            
            // Allow 1 degree tolerance due to rounding
            setpoint_matches = (abs(actual_setpoint_fahrenheit - target_setpoint_fahrenheit) <= 1.0f);
            
            ESP_LOGVV(TAG, "Climate setpoint check: circuit=%u, target=%.1f°F, actual=%.1f°F, matches=%d",
                     circuit_idx, target_setpoint_fahrenheit, actual_setpoint_fahrenheit, setpoint_matches);
        }
        
        // Check heat source if it was changed
        if (it->has_heat_src_change) {
            heat_src_matches = (thermo->heat_src == it->target_heat_src);
            
            ESP_LOGVV(TAG, "Climate heat_src check: circuit=%u, target=%u, actual=%u, matches=%d",
                     circuit_idx, it->target_heat_src, thermo->heat_src, heat_src_matches);
        }
        
        // Check if both changes are confirmed
        if (setpoint_matches && heat_src_matches) {
            ESP_LOGVV(TAG, "Climate confirmed: circuit=%u", circuit_idx);
            
            // Update and publish the climate state
            float water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
            float water_temp_c = (water_temp_f - 32.0f) * 5.0f / 9.0f;
            
            it->climate->current_temperature = water_temp_c;
            it->climate->target_temperature = (thermo->set_point - 32.0f) * 5.0f / 9.0f;
            
            // Update mode based on circuit state
            if (new_state->circuits.active[circuit_idx]) {
                if (thermo->heating) {
                    it->climate->mode = climate::CLIMATE_MODE_HEAT;
                } else {
                    it->climate->mode = climate::CLIMATE_MODE_AUTO;
                }
            } else {
                it->climate->mode = climate::CLIMATE_MODE_OFF;
            }
            
            // Update action
            if (thermo->heating) {
                it->climate->action = climate::CLIMATE_ACTION_HEATING;
            } else if (new_state->circuits.active[circuit_idx]) {
                it->climate->action = climate::CLIMATE_ACTION_IDLE;
            } else {
                it->climate->action = climate::CLIMATE_ACTION_OFF;
            }
            
            it->climate->publish_state();
            it = pending_climates_.erase(it);
        }
        // Check for timeout (e.g., 15 seconds)
        else if (millis() - it->timestamp > 15000) {
            ESP_LOGW(TAG, "Climate confirmation timeout, circuit=%u", circuit_idx);
            
            // Publish current state anyway
            it->climate->publish_state();
            it = pending_climates_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace opnpool
}  // namespace esphome