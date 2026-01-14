/**
 * @file opnpool.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief Implementation of the OpnPool component for ESPHome.
 * 
 * @copyright Copyright (c) 2026, Coert Vonk
 */
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
  
static char const * const TAG = "opnpool";

/**
 * @brief Set up the OpnPool component.
 * 
 * This function initializes the OpnPool state, sets up inter-process communication (IPC) queues,
 * and starts the pool task that handles RS485 communication, datalink layer, and network layer.
 * It also publishes the interface firmware version if available.
 */
void OpnPool::setup() {
    
    ESP_LOGV(TAG, "Setting up OpnPool...");

    opnPoolState_ = new OpnPoolState(this);
    if (!opnPoolState_ || !opnPoolState_->is_valid()) {
        ESP_LOGE(TAG, "Failed to initialize OpnPoolState");
        return;
    }

    ipc_ = new ipc_t{};
    if (ipc_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate IPC structure");
        return;
    }
    
    ipc_->to_pool_q = xQueueCreate(6, sizeof(network_msg_t));
    ipc_->to_main_q = xQueueCreate(10, sizeof(network_msg_t));
    if (!ipc_->to_main_q || !ipc_->to_pool_q) {
        ESP_LOGE(TAG, "Failed to create IPC queues");
        return;
    }

        // spin off a pool_task that handles RS485 communication, datalink layer and network layer
    if (xTaskCreate(&pool_task, "pool_task", 2*4096, this->ipc_, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pool_task");
        return;
    }

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

/**
 * @brief Main loop for the OpnPool component.
 * 
 * This function is called repeatedly by the main ESPHome loop. It handles
 * service requests from the pool by checking the IPC queue for messages
 * from the pool task.
 */
void OpnPool::loop() {

    // don't do any blocking operations here

    this->service_requests_from_pool(this->ipc_);
}

/**
 * @brief Dump the configuration of the OpnPool component.
 * 
 * This function logs the configuration of the OpnPool component, including
 * the RS485 pin assignments and the configuration of all associated climate,
 * switch, sensor, binary sensor, and text sensor components.
 */
void OpnPool::dump_config() {

    ESP_LOGCONFIG(TAG, "OpnPool:");
    ESP_LOGCONFIG(TAG, "  RS485 RX Pin: %u", this->ipc_->config.rs485_pins.rx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 TX Pin: %u", this->ipc_->config.rs485_pins.tx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 Flow Control Pin: %u", this->ipc_->config.rs485_pins.flow_control_pin);

    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(ClimateId::COUNT); idx++) {
        OpnPoolClimate * const climate = this->climates_[idx];
        if (climate != nullptr) {
            climate->dump_config();
        }
    }

    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(SwitchId::COUNT); idx++) {
        OpnPoolSwitch * const sw = this->switches_[idx];
        if (sw != nullptr) {
            sw->dump_config();
        }
    }

    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(SensorId::COUNT); idx++) {
        OpnPoolSensor * const sensor = this->sensors_[idx];
        if (sensor != nullptr) {
            sensor->dump_config();
        }
    }

    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(BinarySensorId::COUNT); idx++) {
        OpnPoolBinarySensor * const binary_sensor = this->binary_sensors_[idx];
        if (binary_sensor != nullptr) {
            binary_sensor->dump_config();
        }
    }

    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(TextSensorId::COUNT); idx++) {
        OpnPoolTextSensor * const text_sensor = this->text_sensors_[idx];
        if (text_sensor != nullptr) {
            text_sensor->dump_config();
        }
    }
}


void OpnPool::update_climates(poolstate_t const * const new_state)
{
    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(ClimateId::COUNT); idx++) {

        OpnPoolClimate * const climate = this->climates_[idx];
        if (climate != nullptr) {
            climate->update_climate(new_state);
        }
    }
}

void OpnPool::update_switches(poolstate_t const * const new_state)
{
    for (uint8_t idx = 0; static_cast<uint8_t>(idx) < static_cast<uint8_t>(SwitchId::COUNT); idx++) {
        
        OpnPoolSwitch * const sw = this->switches_[idx];
        if (sw != nullptr) {
            sw->check_pending_switch(new_state);
        }
    }
}

void OpnPool::update_analog_sensors(poolstate_t const * const new_state)
{
    auto * const air_temp = this->sensors_[static_cast<uint8_t>(SensorId::AIR_TEMP)];
    if (air_temp != nullptr) {
        float air_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)].temp;
        float air_temp_c = (air_temp_f - 32.0f) * 5.0f / 9.0f;
        air_temp->publish_state_if_changed(air_temp_c);    
    }

    auto * const water_temp = this->sensors_[static_cast<uint8_t>(SensorId::WATER_TEMP)];
    if (water_temp != nullptr) {    
        float const water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
        float const water_temp_c = (water_temp_f - 32.0f) * 5.0f / 9.0f;
        water_temp->publish_state_if_changed(water_temp_c);
    }

    auto * const pump_power = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_POWER)];
    if (pump_power != nullptr) {
        pump_power->publish_state_if_changed(new_state->pump.power);
    }

    auto * const pump_flow = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_FLOW)];
    if (pump_flow != nullptr) {
        pump_flow->publish_state_if_changed(new_state->pump.flow);  
    }

    auto * const pump_speed = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_SPEED)];
    if (pump_speed != nullptr) {
        pump_speed->publish_state_if_changed(new_state->pump.speed);
    }

    auto * const pump_error = this->sensors_[static_cast<uint8_t>(SensorId::PUMP_ERROR)];
    if (pump_error != nullptr) {
        pump_error->publish_state_if_changed(new_state->pump.error);
    }

    auto * const chlor_level = this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_LEVEL)];
    if (chlor_level != nullptr) {
        chlor_level->publish_state_if_changed(new_state->chlor.level);
    }

    auto * const chlor_salt = this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_SALT)];
    if (chlor_salt != nullptr) {
        chlor_salt->publish_state_if_changed(new_state->chlor.salt);
    }
}

void OpnPool::update_binary_sensors(poolstate_t const * const new_state)
{
    auto * const pump_running = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::PUMP_RUNNING)];
    if (pump_running != nullptr) {
        pump_running->publish_state_if_changed(new_state->pump.running);
    }

    auto * const mode_service = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_SERVICE)];
    if (mode_service != nullptr) {
        mode_service->publish_state_if_changed(new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::SERVICE)]);
    }

    auto * const mode_temp_inc = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TEMP_INC)];
    if (mode_temp_inc != nullptr) {
        mode_temp_inc->publish_state_if_changed(new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::TEMP_INC)]);
    }

    auto * const mode_freeze = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_FREEZE)];
    if (mode_freeze != nullptr) {
        mode_freeze->publish_state_if_changed(new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::FREEZE_PROT)]);
    }

    auto * const mode_timeout = this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TIMEOUT)];
    if (mode_timeout != nullptr) {
        mode_timeout->publish_state_if_changed(new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_t::TIMEOUT)]);
    }
}

void OpnPool::update_text_sensors(poolstate_t const * const new_state)
{
    auto * const pool_sched = this->text_sensors_[static_cast<uint8_t>(TextSensorId::POOL_SCHED)];
    if (pool_sched != nullptr) {
        static char sched_str[64];
        uint8_t pool_idx = static_cast<uint8_t>(network_pool_circuit_t::POOL);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[pool_idx].start / 60, new_state->scheds[pool_idx].start % 60,
            new_state->scheds[pool_idx].stop / 60, new_state->scheds[pool_idx].stop % 60);
        pool_sched->publish_state_if_changed(sched_str);
    }
    
    auto * const spa_sched = this->text_sensors_[static_cast<uint8_t>(TextSensorId::SPA_SCHED)];
    if (spa_sched != nullptr) {
        static char sched_str[64];
        uint8_t spa_idx = static_cast<uint8_t>(network_pool_circuit_t::SPA);
        snprintf(sched_str, sizeof(sched_str), "%02d:%02d-%02d:%02d",
            new_state->scheds[spa_idx].start / 60, new_state->scheds[spa_idx].start % 60,
            new_state->scheds[spa_idx].stop / 60, new_state->scheds[spa_idx].stop % 60);
        spa_sched->publish_state_if_changed(sched_str);
    }
    
    auto * const pump_mode = this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_MODE)];
    if (pump_mode != nullptr) {
        pump_mode->publish_state_if_changed(
            network_pump_mode_str(static_cast<network_pump_mode_t>(new_state->pump.mode)));
    }

    auto * const pump_state = this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_STATE)];
    if (pump_state != nullptr) {
        pump_state->publish_state_if_changed(
            network_pump_state_str(static_cast<network_pump_state_t>(new_state->pump.state)));
    }

    auto * const chlor_name = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_NAME)];
    if (chlor_name != nullptr) {
        chlor_name->publish_state_if_changed(new_state->chlor.name);
    }
    
    auto * const chlor_status = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_STATUS)];
    if (chlor_status != nullptr) {
        static char const * status_str = poolstate_str_chlor_status_str(new_state->chlor.status);
        chlor_status->publish_state_if_changed(status_str);
    }
    
    auto * const system_time = this->text_sensors_[static_cast<uint8_t>(TextSensorId::SYSTEM_TIME)];
    if (system_time != nullptr) {
        static char time_str[32];
        snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
            new_state->system.tod.date.year, new_state->system.tod.date.month, new_state->system.tod.date.day,
            new_state->system.tod.time.hour, new_state->system.tod.time.minute);
        system_time->publish_state_if_changed(time_str);
    }
    
    auto * const controller_fw = this->text_sensors_[static_cast<uint8_t>(TextSensorId::CONTROLLER_FW)];
    if (controller_fw != nullptr) {
        static char fw_str[16];
        snprintf(fw_str, sizeof(fw_str), "%d.%d", new_state->system.version.major, new_state->system.version.minor);
        controller_fw->publish_state_if_changed(fw_str);
    }
}

void OpnPool::update_all(poolstate_t const * const state)
{
        this->update_climates(state);
        this->update_switches(state);
        this->update_text_sensors(state);
        this->update_analog_sensors(state);
        this->update_binary_sensors(state);
}

    // RS485 setter

void 
OpnPool::set_rs485_pins(uint8_t const rx_pin, uint8_t const tx_pin, uint8_t const flow_control_pin)
{
    if (ipc_) {
        ipc_->config.rs485_pins.rx_pin = rx_pin;
        ipc_->config.rs485_pins.tx_pin = tx_pin;
        ipc_->config.rs485_pins.flow_control_pin = flow_control_pin;
    }
}

    // climate setters      

void
OpnPool::set_pool_climate(OpnPoolClimate * const climate)
{ 
    if (climate != nullptr) {
        climate->set_parent(this);
        climate->set_idx(static_cast<uint8_t>(poolstate_thermo_typ_t::POOL));
        this->climates_[static_cast<uint8_t>(poolstate_thermo_typ_t::POOL)] = climate; 
    }
}

void
OpnPool::set_spa_climate(OpnPoolClimate * const climate)
{ 
    if (climate != nullptr) {
        climate->set_parent(this);
        climate->set_idx(static_cast<uint8_t>(poolstate_thermo_typ_t::SPA));
        this->climates_[static_cast<uint8_t>(poolstate_thermo_typ_t::SPA)] = climate; 
    }
}
    // switch setters

void
OpnPool::set_pool_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::POOL));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::POOL)] = sw; 
    }
}
void
OpnPool::set_spa_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::SPA));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::SPA)] = sw; 
    }
}

void
OpnPool::set_aux1_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX1));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX1)] = sw; 
    }
}

void
OpnPool::set_aux2_switch(OpnPoolSwitch * const sw) 
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX2));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX2)] = sw; 
    }
}
void
OpnPool::set_aux3_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::AUX3));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX3)] = sw; 
    }
}

void
OpnPool::set_feature1_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE1));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE1)] = sw; 
    }
}

void
OpnPool::set_feature2_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE2));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE2)] = sw; 
    }
}

void
OpnPool::set_feature3_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE3));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE3)] = sw; 
    }
}

void
OpnPool::set_feature4_switch(OpnPoolSwitch * const sw)
{ 
    if (sw != nullptr) {
        sw->set_parent(this);
        sw->set_idx(static_cast<uint8_t>(network_pool_circuit_t::FEATURE4));
        this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE4)] = sw; 
    }
}

    // sensor setters

void
OpnPool::set_air_temperature_sensor(OpnPoolSensor * const s) { 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::AIR_TEMP)] = s; 
    }
}

void
OpnPool::set_water_temperature_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::WATER_TEMP)] = s; 
    }
}
void
OpnPool::set_pump_power_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::PUMP_POWER)] = s; 
    }
}

void
OpnPool::set_pump_flow_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::PUMP_FLOW)] = s; 
    }
}
void
OpnPool::set_pump_speed_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::PUMP_SPEED)] = s; 
    }
}

void
OpnPool::set_pump_error_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::PUMP_ERROR)] = s; 
    }
}
void
OpnPool::set_chlorinator_level_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_LEVEL)] = s; 
    }
}

void
OpnPool::set_chlorinator_salt_sensor(OpnPoolSensor * const s)
{ 
    if (s != nullptr) {
        s->set_parent(this);
        this->sensors_[static_cast<uint8_t>(SensorId::CHLOR_SALT)] = s; 
    }
}

    // binary sensor setters

void
OpnPool::set_pump_running_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    if (bs != nullptr) {
        bs->set_parent(this);
        this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::PUMP_RUNNING)] = bs; 
    }
}
void
OpnPool::set_mode_service_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    if (bs != nullptr) {
        bs->set_parent(this);
        this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_SERVICE)] = bs; 
    }
}

void
OpnPool::set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    if (bs != nullptr) {
        bs->set_parent(this);
        this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TEMP_INC)] = bs; 
    }
}
void
OpnPool::set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    if (bs != nullptr) {
        bs->set_parent(this);
        this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_FREEZE)] = bs; 
    }
}

void
OpnPool::set_mode_timeout_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    if (bs != nullptr) {
        bs->set_parent(this);
        this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TIMEOUT)] = bs; 
    }
}
    // text sensor setters

void
OpnPool::set_pool_sched_text_sensor(OpnPoolTextSensor * const ts) 
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::POOL_SCHED)] = ts; 
    }
}
void
OpnPool::set_spa_sched_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::SPA_SCHED)] = ts; 
    }
}

void
OpnPool::set_pump_mode_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_MODE)] = ts; 
    }
}
void
OpnPool::set_pump_state_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_STATE)] = ts; 
    }
}

void
OpnPool::set_chlorinator_name_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_NAME)] = ts; 
    }
}
void
OpnPool::set_chlorinator_status_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLOR_STATUS)] = ts; 
    }
}

void
OpnPool::set_system_time_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::SYSTEM_TIME)] = ts; 
    }
}
void
OpnPool::set_controller_firmware_version_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CONTROLLER_FW)] = ts; 
    }
}

void
OpnPool::set_interface_firmware_version_text_sensor(OpnPoolTextSensor * const ts)
{ 
    if (ts != nullptr) {
        ts->set_parent(this);
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::INTERFACE_FW)] = ts; 
    }
}
}  // namespace opnpool
}  // namespace esphome