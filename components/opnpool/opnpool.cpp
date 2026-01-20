/**
 * @file opnpool.cpp
 * @brief Implementation of the OPNpool component for ESPHome.
 *
 * @details
 * This file contains the implementation of the OPNpool component, which provides
 * integration between an OPNpool interface and the ESPHome ecosystem. The component
 * manages communication with the Pentair pool controller via RS485, processes datalink
 * and network layer messages, and exposes pool state and controls as ESPHome entities.
 *
 * The OPNpool component is responsible for:
 * - Spawning and supervising the pool_task, a FreeRTOS task that handles low-level RS485
 *   communication, datalink protocol parsing, and network message handling.
 * - Updating ESPHome climate, switch, sensor, binary sensor, and text sensor entities
 *   based on the latest pool state.
 * - Providing setter methods for associating ESPHome entities with the OPNpool component.
 *
 * The design leverages modular helper functions for each protocol layer and uses FreeRTOS
 * primitives for task scheduling and inter-task communication. Extensive logging and
 * debug output are provided for troubleshooting and protocol analysis.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. 
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <cstdlib>
#include <esphome/core/log.h>
#include <esphome/core/hal.h>
#include <type_traits>

#include "pool_state_rx.h"
#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "ipc.h"
#include "pool_task.h"
#include "opnpool.h"
#include "to_str.h"

#include "pool_state.h"
#include "opnpool_climate.h"
#include "opnpool_switch.h"
#include "opnpool_sensor.h"
#include "opnpool_binary_sensor.h"
#include "opnpool_text_sensor.h"
#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {
  
static char const * const TAG = "opnpool";


    // helper to only publish if entity exists
template<typename EntityT, typename ValueT>
inline void 
_publish_if(EntityT *entity, ValueT value)
{
    if (entity != nullptr) {
        entity->publish_value_if_changed(value);
    }
}

    // helper to publish scheduled times if entity exists
template<typename StartT, typename StopT>
void 
_publish_schedule_if(OpnPoolTextSensor *sensor, StartT start, StopT stop)
{
    if (sensor != nullptr) {
        char buf[12];  // HH:MM-HH:MM\n
        snprintf(buf, sizeof(buf), "%02d:%02d-%02d:%02d", start / 60, start % 60, stop / 60, stop % 60);
        sensor->publish_value_if_changed(buf);
    }
}

    // helper to publish date and time if entity exists
template<typename TodT>
void 
_publish_date_and_time_if(OpnPoolTextSensor *sensor, TodT tod)
{
    if (sensor != nullptr && tod != nullptr) {
        static char time_str[17];  // 2026-01-15 22:43\n
        snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
            tod->date.year, tod->date.month, tod->date.day,
            tod->time.hour, tod->time.minute);
        sensor->publish_value_if_changed(time_str);
    }
}

    // helper to publish version if entity exists
template<typename VersionT>
void
_publish_version_if(OpnPoolTextSensor *sensor, VersionT version)
{
    if (sensor != nullptr && version != nullptr) {
        static char fw_str[8];  // 2.80\0
        snprintf(fw_str, sizeof(fw_str), "%d.%d", version->major, version->minor);
        sensor->publish_value_if_changed(fw_str);
    }
}

    // helper to only dump_config if entity exists
template<typename EntityT>
inline void 
_dump_if(EntityT *entity)
{
    if (entity != nullptr) {
        entity->dump_config();
    }
}

inline float
_fahrenheit_to_celsius(float f) {
    return (f - 32.0f) * 5.0f / 9.0f;
}

/**
 * @brief Set up the OpnPool component.
 *
 * This function initializes the OpnPool state, sets up inter-process communication (IPC)
 * queues, and starts the pool task that handles RS485 communication, datalink layer, and
 * network layer. It also publishes the interface firmware version if available.
 */
void 
OpnPool::setup() {
    
    ESP_LOGI(TAG, "Setting up OpnPool...");

    poolState_ = new PoolState(this);
    if (!poolState_) {
        ESP_LOGE(TAG, "Failed to initialize PoolState");
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
        ESP_LOGE(TAG, "Failed to create IPC queue(s)");
        return;
    }

        // spin off a pool_task that handles RS485 communication, datalink layer and
        // network layer
    if (xTaskCreate(&pool_task, "pool_task", 2*4096, this->ipc_, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pool_task");
        return;
    }

        // publish interface firmware version
    auto * const interface_firmware = this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::INTERFACE_FIRMWARE)];
    if (interface_firmware != nullptr) {
#ifdef GIT_HASH
            interface_firmware->publish_state(GIT_HASH);
#else
            interface_firmware->publish_state("unknown");
#endif
    }
}

/**
 * @brief Main loop for the OpnPool component.
 *
 * This function is called repeatedly by the main ESPHome loop. It handles service
 * requests from the pool by checking the IPC queue for messages from the pool task.
 * Warning: don't do any blocking operations here.
 */
void
OpnPool::loop() {

    network_msg_t msg = {};

    if (xQueueReceive(ipc_->to_main_q, &msg, 0) == pdPASS) {  // check if a message is available

            // reset global string buffer (as a new cycle begins)
        name_reset_idx();

            // start with new_state being the current state
        poolstate_t new_state;
        poolState_->get(&new_state);

        ESP_LOGVV(TAG, "Handling msg typ=%s", enum_str(msg.typ));

        if (pool_state_rx::update_state(&msg, &new_state) == ESP_OK) {

            if (poolState_->has_changed(&new_state)) {

                poolState_->set(&new_state);

                    // publish this as an update to the HA sensors 
                this->update_climates(&new_state);
                this->update_switches(&new_state);
                this->update_text_sensors(&new_state);
                this->update_analog_sensors(&new_state);
                this->update_binary_sensors(&new_state);
            }
 
            ESP_LOGVV(TAG, "FYI Poolstate changed");
        }
    }
}

/**
 * @brief Dump the configuration of the OpnPool component.
 *
 * This function logs the configuration of the OpnPool component, including the RS485 pin
 * assignments and the configuration of all associated climate, switch, sensor, binary
 * sensor, and text sensor components.
 */
void 
OpnPool::dump_config() {

    ESP_LOGCONFIG(TAG, "OpnPool:");
    ESP_LOGCONFIG(TAG, "  RS485 RX Pin: %u", this->ipc_->config.rs485_pins.rx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 TX Pin: %u", this->ipc_->config.rs485_pins.tx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 Flow Control Pin: %u", this->ipc_->config.rs485_pins.flow_control_pin);

    for (auto idx : magic_enum::enum_values<climate_id_t>()) {
        _dump_if(this->climates_[enum_index(idx)]);
    }   
    for (auto idx : magic_enum::enum_values<switch_id_t>()) {
        _dump_if(this->switches_[enum_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<sensor_id_t>()) {
        _dump_if(this->sensors_[enum_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<binary_sensor_id_t>()) {
        _dump_if(this->binary_sensors_[enum_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<text_sensor_id_t>()) {
        if (idx != text_sensor_id_t::INTERFACE_FIRMWARE) {
            _dump_if(this->text_sensors_[enum_index(idx)]);
        }
    }
}

void 
OpnPool::update_climates(const poolstate_t *new_state)
{
    for (auto climate_id : magic_enum::enum_values<climate_id_t>()) {

        OpnPoolClimate *climate = this->climates_[enum_index(climate_id)];
        if (!climate) continue;

        auto const thermo_typ = climate->get_thermo_typ();
        auto const thermo = &new_state->thermos[enum_index(thermo_typ)];

            // temperatures
        float const current_temp_f = new_state->temps[enum_index(poolstate_temp_typ_t::WATER)].temp;
        float const current_temp_c = fahrenheit_to_celsius(current_temp_f);
        float const target_temp_c = fahrenheit_to_celsius(thermo->set_point_in_f);

            // mode
        uint8_t switch_idx = (thermo_typ == poolstate_thermo_typ_t::POOL)
                           ? enum_index(network_pool_circuit_t::POOL)
                           : enum_index(network_pool_circuit_t::SPA);

        climate::ClimateMode mode = new_state->circuits.active[switch_idx]
                                  ? climate::CLIMATE_MODE_HEAT
                                  : climate::CLIMATE_MODE_OFF;

            // custom preset
        auto const custom_preset = enum_str(thermo->heat_src);

            // action
        climate::ClimateAction action = thermo->heating
            ? climate::CLIMATE_ACTION_HEATING
            : (mode == climate::CLIMATE_MODE_OFF
                ? climate::CLIMATE_ACTION_OFF
                : climate::CLIMATE_ACTION_IDLE);

        climate->publish_value_if_changed(current_temp_c, target_temp_c, mode, custom_preset, action);
    }
}

void
OpnPool::update_switches(const poolstate_t *state)
{
    for (auto switch_id : magic_enum::enum_values<switch_id_t>()) {

        OpnPoolSwitch *sw = this->switches_[enum_index(switch_id)];
        if (!sw) continue;

        auto const circuit = switch_id_to_network_circuit(switch_id);
        bool const is_active = state->circuits.active[enum_index(circuit)];

        sw->publish_value_if_changed(is_active);
    }
}

void
OpnPool::update_analog_sensors(poolstate_t const * const new_state)
{
    auto * const air_temp = this->sensors_[static_cast<uint8_t>(sensor_id_t::AIR_TEMPERATURE)];
    if (air_temp != nullptr) {
        float const air_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)].temp;
        float air_temp_c = fahrenheit_to_celsius(air_temp_f);
        air_temp_c = std::round(air_temp_c * 10.0f) / 10.0f;
        air_temp->publish_value_if_changed(air_temp_c);    
    }
    auto * const water_temperature = this->sensors_[static_cast<uint8_t>(sensor_id_t::WATER_TEMPERATURE)];
    if (water_temperature != nullptr) {    
        float const water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
        float water_temp_c = fahrenheit_to_celsius(water_temp_f);
        water_temp_c = std::round(water_temp_c * 10.0f) / 10.0f;
        water_temperature->publish_value_if_changed(water_temp_c);

    }
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_POWER)],        
        new_state->pump.power
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_FLOW)],         
        new_state->pump.flow
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_SPEED)],        
        new_state->pump.speed
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_ERROR)],        
        new_state->pump.error
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::CHLORINATOR_LEVEL)], 
        new_state->chlor.level
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::CHLORINATOR_SALT)],  
        new_state->chlor.salt
    );
}

void 
OpnPool::update_binary_sensors(poolstate_t const * const new_state)
{
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::PUMP_RUNNING)],           
        new_state->pump.running
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_SERVICE)],           
        new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::SERVICE)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_TEMPERATURE_INC)],   
        new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::TEMP_INC)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_FREEZE_PROTECTION)], 
        new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::FREEZE_PROT)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_TIMEOUT)],           
        new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::TIMEOUT)]
    );
}

void 
OpnPool::update_text_sensors(poolstate_t const * const new_state)
{
    _publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::POOL_SCHED)],
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::POOL)].start,
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::POOL)].stop
    );
    _publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::SPA_SCHED)],
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::SPA)].start,
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::SPA)].stop
    );
    _publish_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::PUMP_MODE)], 
        enum_str(new_state->pump.mode)
    );    
    _publish_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::PUMP_STATE)],
        enum_str(new_state->pump.state)
    );
    _publish_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CHLORINATOR_NAME)], 
        new_state->chlor.name
    );
    _publish_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CHLORINATOR_STATUS)],
        enum_str(new_state->chlor.status)
    );
    _publish_date_and_time_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::SYSTEM_TIME)],
        &new_state->system.tod
    );
    _publish_version_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CONTROLLER_FIRMWARE)],
        &new_state->system.version
    );
}

    // setters

void 
OpnPool::set_rs485_pins(uint8_t const rx_pin, uint8_t const tx_pin, uint8_t const flow_control_pin)
{
    if (ipc_) {
        ipc_->config.rs485_pins.rx_pin = rx_pin;
        ipc_->config.rs485_pins.tx_pin = tx_pin;
        ipc_->config.rs485_pins.flow_control_pin = flow_control_pin;
    }
}

void
OpnPool::set_pool_climate(OpnPoolClimate * const climate)
{ 
    this->climates_[static_cast<uint8_t>(poolstate_thermo_typ_t::POOL)] = climate; 
}

void
OpnPool::set_spa_climate(OpnPoolClimate * const climate)
{ 
    this->climates_[static_cast<uint8_t>(poolstate_thermo_typ_t::SPA)] = climate; 
}

void
OpnPool::set_pool_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::POOL)] = sw; 
}

void
OpnPool::set_spa_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::SPA)] = sw; 
}

void
OpnPool::set_aux1_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX1)] = sw; 
}

void
OpnPool::set_aux2_switch(OpnPoolSwitch * const sw) 
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX2)] = sw; 
}

void
OpnPool::set_aux3_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::AUX3)] = sw; 
}

void
OpnPool::set_feature1_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE1)] = sw; 
}

void
OpnPool::set_feature2_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE2)] = sw;
}

void
OpnPool::set_feature3_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE3)] = sw; 
}

void
OpnPool::set_feature4_switch(OpnPoolSwitch * const sw)
{ 
    this->switches_[static_cast<uint8_t>(network_pool_circuit_t::FEATURE4)] = sw; 
}

void
OpnPool::set_air_temperature_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::AIR_TEMPERATURE)] = s; 
}

void
OpnPool::set_water_temperature_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::WATER_TEMPERATURE)] = s; 
}

void
OpnPool::set_pump_power_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::PUMP_POWER)] = s; 
}

void
OpnPool::set_pump_flow_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::PUMP_FLOW)] = s; 
}

void
OpnPool::set_pump_speed_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::PUMP_SPEED)] = s; 
}

void
OpnPool::set_pump_error_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::PUMP_ERROR)] = s; 
}

void
OpnPool::set_chlorinator_level_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::CHLORINATOR_LEVEL)] = s; 
}

void
OpnPool::set_chlorinator_salt_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[enum_index(sensor_id_t::CHLORINATOR_SALT)] = s; 
}

void
OpnPool::set_pump_running_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[enum_index(binary_sensor_id_t::PUMP_RUNNING)] = bs; 
}

void
OpnPool::set_mode_service_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[enum_index(binary_sensor_id_t::MODE_SERVICE)] = bs; 
}

void
OpnPool::set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[enum_index(binary_sensor_id_t::MODE_TEMPERATURE_INC)] = bs; 
}

void
OpnPool::set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[enum_index(binary_sensor_id_t::MODE_FREEZE_PROTECTION)] = bs; 
}

void
OpnPool::set_mode_timeout_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[enum_index(binary_sensor_id_t::MODE_TIMEOUT)] = bs; 
}

void
OpnPool::set_pool_sched_text_sensor(OpnPoolTextSensor * const ts) 
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::POOL_SCHED)] = ts; 
}

void
OpnPool::set_spa_sched_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::SPA_SCHED)] = ts; 
}

void
OpnPool::set_pump_mode_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::PUMP_MODE)] = ts; 
}

void
OpnPool::set_pump_state_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::PUMP_STATE)] = ts; 
}

void
OpnPool::set_chlorinator_name_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::CHLORINATOR_NAME)] = ts; 
}

void
OpnPool::set_chlorinator_status_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::CHLORINATOR_STATUS)] = ts; 
}

void
OpnPool::set_system_time_text_sensor(OpnPoolTextSensor * const ts)
{ 
     this->text_sensors_[enum_index(text_sensor_id_t::SYSTEM_TIME)] = ts;  
}

void
OpnPool::set_controller_firmware_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::CONTROLLER_FIRMWARE)] = ts; 
}

void
OpnPool::set_interface_firmware_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[enum_index(text_sensor_id_t::INTERFACE_FIRMWARE)] = ts; 
}

}  // namespace opnpool
}  // namespace esphome