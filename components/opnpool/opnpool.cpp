#include <time.h>
#include <esp_system.h>
#include <cstdlib>
#include <esphome/core/log.h>
#include <esphome/core/hal.h>

#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "ipc.h"
#include "pool_task.h"
#include "opnpool.h"

#include "opnpool_state.h"
#include "opnpool_climate.h"
#include "opnpool_switch.h"
#include "opnpool_sensor.h"
#include "opnpool_binary_sensor.h"
#include "opnpool_text_sensor.h"

namespace esphome {
namespace opnpool {
  
static const char *const TAG = "opnpool";


void OpnPool::setup() {
    
    ESP_LOGV(TAG, "Setting up OpnPool...");

    opnPoolState_ = new OpnPoolState(this);
    assert(opnPoolState_->is_valid());

    ipc_ = new ipc_t{};
    assert(ipc_);

    ipc_->to_pool_q = xQueueCreate(6, sizeof(network_msg_t));
    ipc_->to_main_q = xQueueCreate(10, sizeof(network_msg_t));
    assert(ipc_->to_main_q && ipc_->to_pool_q);

        // spin off a pool_task that handles RS485 communication, datalink layer and network layer
    xTaskCreate(&pool_task, "pool_task", 2*4096, this->ipc_, 3, NULL);

        // publish interface firmware version
    auto * const interface_fw = this->text_sensors_[static_cast<uint8_t>(TextSensorId::INTERFACE_FW)];
    if (interface_fw != nullptr) {
        #ifdef INTERFACE_FW_VERSION
            interface_fw->publish_state(INTERFACE_FW_VERSION);
        #else
            interface_fw->publish_state("unknown");
        #endif
    }
}

void
OpnPool::service_requests_from_pool(ipc_t const * const ipc)
{
    network_msg_t msg = {};

    if (xQueueReceive(ipc->to_main_q, &msg, 0) == pdPASS) {  // don't block, just check if a message is available

        ESP_LOGVV(TAG, "Handling msg typ=%s", ipc_to_home_typ_str(msg.typ));

        if (opnPoolState_->rx_update(&msg) == ESP_OK) {

            ESP_LOGVV(TAG, "Poolstate changed");

        }
    }
}

void OpnPool::loop() {  //  this will be called repeatedly by the main ESPHome loop

    // don't do any blocking operations here

    this->service_requests_from_pool(this->ipc_);
}

void OpnPool::dump_config() {

    ESP_LOGCONFIG(TAG, "OpnPool:");
    ESP_LOGCONFIG(TAG, "  RS485 RX Pin: %u", this->ipc_->config.rs485_pins.rx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 TX Pin: %u", this->ipc_->config.rs485_pins.tx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 Flow Control Pin: %u", this->ipc_->config.rs485_pins.flow_control_pin);
}

void OpnPool::update_analog_sensors(const poolstate_t *new_state) {

    auto *air_temp = this->sensors_[static_cast<uint8_t>(SensorId::AIR_TEMP)];
    if (air_temp != nullptr) {
        float air_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)].temp;
        air_temp->publish_state_if_changed(air_temp_f);
    }
    
    auto *water_temp = this->sensors_[static_cast<uint8_t>(SensorId::WATER_TEMP)];
    if (water_temp != nullptr) {
        float water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
        water_temp->publish_state_if_changed(water_temp_f);
    }
    
    auto *pump_power = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_POWER)];
    if (pump_power != nullptr) {
        pump_power->publish_state_if_changed(new_state->pump.power);
    }
    
    auto *pump_flow = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_FLOW)];
    if (pump_flow != nullptr) {
        pump_flow->publish_state_if_changed(new_state->pump.flow);
    }
    
    auto *pump_speed = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_SPEED)];
    if (pump_speed != nullptr) {
        pump_speed->publish_state_if_changed(new_state->pump.speed);
    }
    
    auto *pump_error = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_ERROR)];
    if (pump_error != nullptr) {
        pump_error->publish_state_if_changed(new_state->pump.error);
    }

    auto *chlor_level = this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_LEVEL)];
    if (chlor_level != nullptr) {
        chlor_level->publish_state_if_changed(new_state->chlor.level);
    }

    auto *chlor_salt = this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_SALT)];
    if (chlor_salt != nullptr) {
        chlor_salt->publish_state_if_changed(new_state->chlor.salt);
    }
}

void OpnPool::update_binary_sensors(const poolstate_t *new_state) {

    auto *pump_running = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::PUMP_RUNNING)];
    if (pump_running != nullptr) {
        pump_running->publish_state_if_changed(new_state->pump.running);
    }
    
    auto *mode_service = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_SERVICE)];
    if (mode_service != nullptr) {
        mode_service->publish_state_if_changed(
            new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::SERVICE)]);
    }
    
    auto *mode_temp_inc = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TEMP_INC)];
    if (mode_temp_inc != nullptr) {
        mode_temp_inc->publish_state_if_changed(
            new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::TEMP_INC)]);
    }
    
    auto *mode_freeze = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_FREEZE)];
    if (mode_freeze != nullptr) {
        mode_freeze->publish_state_if_changed(
            new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::FREEZE_PROT)]);
    }
    
    auto *mode_timeout = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TIMEOUT)];
    if (mode_timeout != nullptr) {
        mode_timeout->publish_state_if_changed(
            new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::TIMEOUT)]);
    }
}

void OpnPool::update_text_sensors(const poolstate_t *new_state) {
    
        // schedule
    auto *pool_sched = this->text_sensors_[static_cast<uint8_t>(TextSensorId::POOL_SCHED)];
    if (pool_sched != nullptr) {
        char sched_str[64];
        uint8_t pool_idx = static_cast<uint8_t>(network_pool_circuit_t::POOL);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[pool_idx].start / 60, new_state->scheds[pool_idx].start % 60,
            new_state->scheds[pool_idx].stop / 60, new_state->scheds[pool_idx].stop % 60);
        pool_sched->publish_state_if_changed(sched_str);
    }
    
    auto *spa_sched = this->text_sensors_[static_cast<uint8_t>(TextSensorId::SPA_SCHED)];
    if (spa_sched != nullptr) {
        char sched_str[64];
        uint8_t spa_idx = static_cast<uint8_t>(network_pool_circuit_t::SPA);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[spa_idx].start / 60, new_state->scheds[spa_idx].start % 60,
            new_state->scheds[spa_idx].stop / 60, new_state->scheds[spa_idx].stop % 60);
        spa_sched->publish_state_if_changed(sched_str);
    }
    
        // pump
    auto *pump_mode = this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_MODE)];
    if (pump_mode != nullptr) {
        pump_mode->publish_state_if_changed(
            network_pump_mode_str(static_cast<network_pump_mode_t>(new_state->pump.mode)));
    }

    auto *pump_state = this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_STATE)];
    if (pump_state != nullptr) {
        pump_state->publish_state_if_changed(
            network_pump_state_str(static_cast<network_pump_state_t>(new_state->pump.state)));
    }

        // chlorinator
    auto *chlor_name = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_NAME)];
    if (chlor_name != nullptr) {
        chlor_name->publish_state_if_changed(new_state->chlor.name);
    }
    
    auto *chlor_status = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_STATUS)];
    if (chlor_status != nullptr) {
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
        chlor_status->publish_state_if_changed(status_str);
    }
    
        // system time
    auto *system_time = this->text_sensors_[static_cast<uint8_t>(TextSensorId::SYSTEM_TIME)];
    if (system_time != nullptr) {
        char time_str[32];
        snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
            new_state->system.tod.date.year, new_state->system.tod.date.month, new_state->system.tod.date.day,
            new_state->system.tod.time.hour, new_state->system.tod.time.minute);
        system_time->publish_state_if_changed(time_str);
    }
    
        // firmware version
    auto *controller_fw = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CONTROLLER_FW)];
    if (controller_fw != nullptr) {
        char fw_str[16];
        snprintf(fw_str, sizeof(fw_str), "%d.%d", new_state->system.version.major, new_state->system.version.minor);
        controller_fw->publish_state_if_changed(fw_str);
    }
}

void OpnPool::update_climates(const poolstate_t *new_state) {
    
        // update all thermostats
    for (uint8_t idx = 0; idx < POOLSTATE_THERMO_TYP_COUNT; idx++) {

        OpnPoolClimate *heater = this->heaters_[idx];
        if (heater != nullptr) {
            heater->update_climate(new_state);
        }
    }
}

void OpnPool::update_switches(const poolstate_t *new_state) 
{
    for (uint8_t idx = 0; idx < NETWORK_POOL_CIRCUIT_COUNT; idx++) {
        
        OpnPoolSwitch * const sw = this->switches_[idx];
        if (sw != nullptr) {
            sw->check_pending_switch(new_state);
        }
    }
}

void OpnPool::update_all(const poolstate_t * state) {

        this->update_climates(state);
        this->update_switches(state);
        this->update_text_sensors(state);
        this->update_analog_sensors(state);
        this->update_binary_sensors(state);
}

    // RS485 configuration

void 
OpnPool::set_rs485_pins(uint8_t rx_pin, uint8_t tx_pin, uint8_t flow_control_pin) {
    if (ipc_) {
        ipc_->config.rs485_pins.rx_pin = rx_pin;
        ipc_->config.rs485_pins.tx_pin = tx_pin;
        ipc_->config.rs485_pins.flow_control_pin = flow_control_pin;
    }
}

    // climate setters        
void
OpnPool::set_pool_heater(OpnPoolClimate *climate)
{ 
    this->heaters_[static_cast<uint8_t>(poolstate_thermo_typ_t::POOL)] = climate; 
        if (climate) { climate->set_idx(static_cast<uint8_t>(poolstate_thermo_typ_t::POOL)); climate->set_parent(this); }
}

void
OpnPool::set_spa_heater(OpnPoolClimate *climate)
{ 
    this->heaters_[static_cast<uint8_t>(poolstate_thermo_typ_t::SPA)] = climate; 
    if (climate) { climate->set_idx(static_cast<uint8_t>(poolstate_thermo_typ_t::SPA)); climate->set_parent(this); }
}

    // switch setters
void
OpnPool::set_pool_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::POOL)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::POOL)); sw->set_parent(this); }
}
void
OpnPool::set_spa_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::SPA)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::SPA)); sw->set_parent(this); }
}
void
OpnPool::set_aux1_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX1)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX1)); sw->set_parent(this); }
}
void
OpnPool::set_aux2_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX2)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX2)); sw->set_parent(this); }
}
void
OpnPool::set_aux3_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX3)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX3)); sw->set_parent(this); }
}
void
OpnPool::set_feature1_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE1)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE1)); sw->set_parent(this); }
}
void
OpnPool::set_feature2_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE2)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE2)); sw->set_parent(this); }
}
void
OpnPool::set_feature3_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE3)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE3)); sw->set_parent(this); }
}
void
OpnPool::set_feature4_switch(OpnPoolSwitch *sw) { 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE4)] = sw; 
    if (sw) { sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE4)); sw->set_parent(this); }
}

    // sensor setters
void
OpnPool::set_air_temperature_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::AIR_TEMP)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_water_temperature_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::WATER_TEMP)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_pump_power_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::PUMP_POWER)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_pump_flow_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::PUMP_FLOW)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_pump_speed_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::PUMP_SPEED)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_pump_error_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::PUMP_ERROR)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_chlorinator_level_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_LEVEL)] = s; 
    if (s) s->set_parent(this);
}
void
OpnPool::set_chlorinator_salt_sensor(OpnPoolSensor *s) { 
    this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_SALT)] = s; 
    if (s) s->set_parent(this);
}

    // binary sensor setters
void
OpnPool::set_pump_running_binary_sensor(OpnPoolBinarySensor *bs) { 
    this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::PUMP_RUNNING)] = bs; 
    if (bs) bs->set_parent(this);
}
void
OpnPool::set_mode_service_binary_sensor(OpnPoolBinarySensor *bs) { 
    this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_SERVICE)] = bs; 
    if (bs) bs->set_parent(this);
}
void
OpnPool::set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor *bs) { 
    this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TEMP_INC)] = bs; 
    if (bs) bs->set_parent(this);
}
void
OpnPool::set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor *bs) { 
    this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_FREEZE)] = bs; 
    if (bs) bs->set_parent(this);
}
void
OpnPool::set_mode_timeout_binary_sensor(OpnPoolBinarySensor *bs) { 
    this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TIMEOUT)] = bs; 
    if (bs) bs->set_parent(this);
}

    // text sensor setters
void
OpnPool::set_pool_sched_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::POOL_SCHED)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_spa_sched_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::SPA_SCHED)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_aux1_sched_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::AUX1_SCHED)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_aux2_sched_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::AUX2_SCHED)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_system_time_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::SYSTEM_TIME)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_controller_firmware_version_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::CONTROLLER_FW)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_interface_firmware_version_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::INTERFACE_FW)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_pump_mode_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_MODE)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_pump_state_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_STATE)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_chlorinator_name_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_NAME)] = ts; 
    if (ts) ts->set_parent(this);
}
void
OpnPool::set_chlorinator_status_text_sensor(OpnPoolTextSensor *ts) { 
    this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_STATUS)] = ts; 
    if (ts) ts->set_parent(this);
}

}  // namespace opnpool
}  // namespace esphome