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

#include "pool_state.h"
#include "opnpool_climate.h"
#include "opnpool_switch.h"
#include "opnpool_sensor.h"
#include "opnpool_binary_sensor.h"
#include "opnpool_text_sensor.h"

namespace esphome {
namespace opnpool {
  
static char const * const TAG = "opnpool";


    // helper to convert enum class to its underlying type
template<typename E>
constexpr auto to_index(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

    // helper to convert Fahrenheit to Celsius
constexpr float fahrenheit_to_celsius(float f)
{
    return (f - 32.0f) * 5.0f / 9.0f;
}

    // helper to only publish if entity exists
template<typename EntityT, typename ValueT>
inline void publish_if(EntityT *entity, ValueT value)
{
    if (entity != nullptr) {
        entity->publish_value_if_changed(value);
    }
}

    // helper to publish scheduled times if entity exists
template<typename StartT, typename StopT>
void publish_schedule_if(OpnPoolTextSensor *sensor, StartT start, StopT stop)
{
    if (sensor != nullptr) {
        char buf[12];  // HH:MM-HH:MM\n
        snprintf(buf, sizeof(buf), "%02d:%02d-%02d:%02d", start / 60, start % 60, stop / 60, stop % 60);
        sensor->publish_value_if_changed(buf);
    }
}

    // helper to publish date and time if entity exists
template<typename TodT>
void publish_date_and_time_if(OpnPoolTextSensor *sensor, TodT tod)
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
void publish_version_if(OpnPoolTextSensor *sensor, VersionT version)
{
    if (sensor != nullptr && version != nullptr) {
        static char fw_str[8];  // 2.80\0
        snprintf(fw_str, sizeof(fw_str), "%d.%d", version->major, version->minor);
        sensor->publish_value_if_changed(fw_str);
    }
}

    // helper to only dump_config if entity exists
template<typename EntityT>
inline void dump_if(EntityT *entity)
{
    if (entity != nullptr) {
        entity->dump_config();
    }
}


/**
 * @brief Set up the OpnPool component.
 *
 * This function initializes the OpnPool state, sets up inter-process communication (IPC)
 * queues, and starts the pool task that handles RS485 communication, datalink layer, and
 * network layer. It also publishes the interface firmware version if available.
 */
void OpnPool::setup() {
    
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
    auto * const interface_firmware = this->text_sensors_[static_cast<uint8_t>(TextSensorId::INTERFACE_FIRMWARE)];
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
void OpnPool::loop() {

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
void OpnPool::dump_config() {

    ESP_LOGCONFIG(TAG, "OpnPool:");
    ESP_LOGCONFIG(TAG, "  RS485 RX Pin: %u", this->ipc_->config.rs485_pins.rx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 TX Pin: %u", this->ipc_->config.rs485_pins.tx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 Flow Control Pin: %u", this->ipc_->config.rs485_pins.flow_control_pin);

    for (auto idx : magic_enum::enum_values<ClimateId>()) {
        dump_if(this->climates_[to_index(idx)]);
    }   
    for (auto idx : magic_enum::enum_values<SwitchId>()) {
        dump_if(this->switches_[to_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<SensorId>()) {
        dump_if(this->sensors_[to_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<BinarySensorId>()) {
        dump_if(this->binary_sensors_[to_index(idx)]);
    }
    for (auto idx : magic_enum::enum_values<TextSensorId>()) {
        if (idx == TextSensorId::INTERFACE_FIRMWARE) {
            continue;
        }
        dump_if(this->text_sensors_[to_index(idx)]);
    }
}

void OpnPool::update_climates(poolstate_t const * const new_state)
{
    for (auto idx : magic_enum::enum_values<ClimateId>()) {

        ESP_LOGVV(TAG, "Updating climate[%u]", to_index(idx));

        OpnPoolClimate * const climate = this->climates_[to_index(idx)];
        if (climate != nullptr) {
            climate->update_climate(new_state);
        }
    }
}

void OpnPool::update_switches(poolstate_t const * const new_state)
{
    for (auto idx : magic_enum::enum_values<SwitchId>()) {
        
        OpnPoolSwitch * const sw = this->switches_[to_index(idx)];
        if (sw != nullptr) {
            sw->update_switch(new_state);
        }
    }
}

void OpnPool::update_analog_sensors(poolstate_t const * const new_state)
{
    auto * const air_temp = this->sensors_[static_cast<uint8_t>(SensorId::AIR_TEMPERATURE)];
    if (air_temp != nullptr) {
        float air_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)].temp;
        float air_temp_c = fahrenheit_to_celsius(air_temp_f);
        air_temp->publish_value_if_changed(air_temp_c);    
    }

    auto * const water_temperature = this->sensors_[static_cast<uint8_t>(SensorId::WATER_TEMPERATURE)];
    if (water_temperature != nullptr) {    
        float const water_temp_f = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
        float const water_temp_c = fahrenheit_to_celsius(water_temp_f);
        water_temperature->publish_value_if_changed(water_temp_c);
    }

    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::PUMP_POWER)], new_state->pump.power);
    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::PUMP_FLOW)],         new_state->pump.flow);
    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::PUMP_SPEED)],        new_state->pump.speed);
    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::PUMP_ERROR)],        new_state->pump.error);
    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::CHLORINATOR_LEVEL)], new_state->chlor.level);
    publish_if(this->sensors_[static_cast<uint8_t>(SensorId::CHLORINATOR_SALT)],  new_state->chlor.salt);
}

void OpnPool::update_binary_sensors(poolstate_t const * const new_state)
{
    publish_if(this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::PUMP_RUNNING)],           new_state->pump.running);
    publish_if(this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_SERVICE)],           new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::SERVICE)]);
    publish_if(this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TEMPERATURE_INC)],   new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::TEMP_INC)]);
    publish_if(this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_FREEZE_PROTECTION)], new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::FREEZE_PROT)]);
    publish_if(this->binary_sensors_[static_cast<uint8_t>(BinarySensorId::MODE_TIMEOUT)],           new_state->modes.is_set[static_cast<uint8_t>(network_pool_mode_bits_t::TIMEOUT)]);
}

void OpnPool::update_text_sensors(poolstate_t const * const new_state)
{
    publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::POOL_SCHED)],
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::POOL)].start,
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::POOL)].stop
    );
    publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::SPA_SCHED)],
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::SPA)].start,
        new_state->scheds[static_cast<uint8_t>(network_pool_circuit_t::SPA)].stop
    );
    publish_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_MODE)], 
        enum_str(new_state->pump.mode)
    );    
    publish_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::PUMP_STATE)],
        enum_str(new_state->pump.state)
    );
    publish_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLORINATOR_NAME)], 
        new_state->chlor.name
    );
    publish_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CHLORINATOR_STATUS)],
        enum_str(new_state->chlor.status)
    );
    publish_date_and_time_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::SYSTEM_TIME)],
        &new_state->system.tod
    );
    publish_version_if(
        this->text_sensors_[static_cast<uint8_t>(TextSensorId::CONTROLLER_FIRMWARE)],
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
    this->sensors_[to_index(SensorId::AIR_TEMPERATURE)] = s; 
}

void
OpnPool::set_water_temperature_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::WATER_TEMPERATURE)] = s; 
}

void
OpnPool::set_pump_power_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::PUMP_POWER)] = s; 
}

void
OpnPool::set_pump_flow_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::PUMP_FLOW)] = s; 
}

void
OpnPool::set_pump_speed_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::PUMP_SPEED)] = s; 
}

void
OpnPool::set_pump_error_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::PUMP_ERROR)] = s; 
}
void
OpnPool::set_chlorinator_level_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::CHLORINATOR_LEVEL)] = s; 
}

void
OpnPool::set_chlorinator_salt_sensor(OpnPoolSensor * const s)
{ 
    this->sensors_[to_index(SensorId::CHLORINATOR_SALT)] = s; 
}

void
OpnPool::set_pump_running_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[to_index(BinarySensorId::PUMP_RUNNING)] = bs; 
}

void
OpnPool::set_mode_service_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[to_index(BinarySensorId::MODE_SERVICE)] = bs; 
}

void
OpnPool::set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[to_index(BinarySensorId::MODE_TEMPERATURE_INC)] = bs; 
}

void
OpnPool::set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[to_index(BinarySensorId::MODE_FREEZE_PROTECTION)] = bs; 
}

void
OpnPool::set_mode_timeout_binary_sensor(OpnPoolBinarySensor * const bs)
{ 
    this->binary_sensors_[to_index(BinarySensorId::MODE_TIMEOUT)] = bs; 
}

void
OpnPool::set_pool_sched_text_sensor(OpnPoolTextSensor * const ts) 
{ 
    this->text_sensors_[to_index(TextSensorId::POOL_SCHED)] = ts; 
}

void
OpnPool::set_spa_sched_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::SPA_SCHED)] = ts; 
}

void
OpnPool::set_pump_mode_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::PUMP_MODE)] = ts; 
}

void
OpnPool::set_pump_state_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::PUMP_STATE)] = ts; 
}

void
OpnPool::set_chlorinator_name_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::CHLORINATOR_NAME)] = ts; 
}

void
OpnPool::set_chlorinator_status_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::CHLORINATOR_STATUS)] = ts; 
}

void
OpnPool::set_system_time_text_sensor(OpnPoolTextSensor * const ts)
{ 
     this->text_sensors_[to_index(TextSensorId::SYSTEM_TIME)] = ts;  
}

void
OpnPool::set_controller_firmware_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::CONTROLLER_FIRMWARE)] = ts; 
}

void
OpnPool::set_interface_firmware_text_sensor(OpnPoolTextSensor * const ts)
{ 
    this->text_sensors_[to_index(TextSensorId::INTERFACE_FIRMWARE)] = ts; 
}

}  // namespace opnpool
}  // namespace esphome