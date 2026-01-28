/**
 * @file opnpool.cpp
 * @brief Implementation of the OPNpool component for ESPHome.
 *
 * @details
 * This file implements the OPNpool component, providing seamless, bidirectional
 * integration between the pool controller and the ESPHome ecosystem. It follows a
 * publish/subscribe model:
 *   - Publishes changes in PoolState to ESPHome climate, switch, sensor, binary sensor,
 *     and text sensor entities, ensuring Home Assistant always reflects the latest pool
 *     state and equipment status.
 *   - Subscribes to ESPHome entity state changes, enacting requests to set switches and
 *     climate controls on the physical pool equipment.
 *   - Spawns and supervises the pool_task (FreeRTOS), which manages low-level RS485
 *     communication, protocol parsing, and network message processing for robust hardware
 *     integration.
 *
 * The design emphasizes modularity, extensibility, and maintainability, leveraging helper
 * functions for protocol abstraction and FreeRTOS primitives for robust task scheduling
 * and inter-task communication. Extensive logging and debug output are provided for
 * diagnostics, troubleshooting, and protocol analysis.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. 
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/log.h>
#include <esphome/core/hal.h>

#include "poolstate_rx.h"
#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "ipc.h"
#include "pool_task.h"
#include "opnpool.h"
#include "to_str.h"
#include "poolstate.h"
#include "opnpool_climate.h"
#include "opnpool_switch.h"
#include "opnpool_sensor.h"
#include "opnpool_binary_sensor.h"
#include "opnpool_text_sensor.h"
#include "opnpool_ids.h"
#include "network.h"
#include "network_msg.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {
  
constexpr char TAG[] = "opnpool";

constexpr uint32_t    POOL_TASK_STACK_SIZE = 2 * 4096;
constexpr UBaseType_t TO_POOL_QUEUE_LEN = 6;
constexpr UBaseType_t TO_MAIN_QUEUE_LEN = 10;

    // helper to only dump_config if entity exists
template<typename EntityT>
static void 
_dump_if(EntityT * const entity)
{
    if (entity != nullptr) {
        entity->dump_config();
    }
}

    // helper to only publish if entity exists
template<typename EntityT, typename ValueT>
static void 
_publish_if(EntityT * const entity, ValueT const base)
{
    if (entity != nullptr && base.valid) {
        entity->publish_value_if_changed(base.value);
    }
}

    // helper to only publish if entity exists
template<typename EntityT, typename ValueT>
static void 
_publish_enum_if(EntityT * const entity, ValueT const base)
{
    if (entity != nullptr && base.valid) {
        entity->publish_value_if_changed(enum_str(base.value));
    }
}

    // helper to publish scheduled times if entity exists
static void
_publish_schedule_if(OpnPoolTextSensor * const sensor, poolstate_sched_t const * const sched)
{
    if (sensor == nullptr || !sched->valid) {
        return;
    }
    char buf[16];  // HH:MM-HH:MM\0

    snprintf(buf, sizeof(buf), "%02d:%02d-%02d:%02d",
             sched->start / 60, sched->start % 60,
             sched->stop / 60, sched->stop % 60);

    sensor->publish_value_if_changed(buf);
}

    // helper to publish date and time if entity exists
static void
_publish_date_and_time_if(OpnPoolTextSensor * const sensor, poolstate_tod_t const * const tod)
{
    if (sensor == nullptr || tod == nullptr || !tod->time.valid) {
        return;
    }
    static char time_str[22];  // 2026-01-15 22:43\0

    if (tod->date.valid) {
        snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d",
                 tod->date.year, tod->date.month, tod->date.day,
                 tod->time.hour, tod->time.minute);
    } else {
        snprintf(time_str, sizeof(time_str), "%02d:%02d",
                 tod->time.hour, tod->time.minute);
    }
    sensor->publish_value_if_changed(time_str);
}

    // helper to publish version if entity exists
static void
_publish_version_if(OpnPoolTextSensor * const sensor, poolstate_version_t const * const version)
{
    if (sensor != nullptr && version != nullptr) {
        static char fw_str[8];  // 2.80\0
        snprintf(fw_str, sizeof(fw_str), "%d.%d", version->major, version->minor);
        sensor->publish_value_if_changed(fw_str);
    }
}

[[nodiscard]] static uint8_t
_thermo_typ_to_pool_circuit_idx(poolstate_thermo_typ_t const thermo_typ)
{
    switch (thermo_typ) {
        case poolstate_thermo_typ_t::POOL:
            return enum_index(network_pool_circuit_t::POOL);
        case poolstate_thermo_typ_t::SPA:
            return enum_index(network_pool_circuit_t::SPA);
        default:
            ESP_LOGE(TAG, "Invalid thermo_typ: %d", static_cast<int>(thermo_typ));
            return -1;  // invalid index, will cause out-of-bounds access if used
    }
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
    ipc_->config.rs485_pins = rs485_pins_;
    ipc_->to_pool_q = xQueueCreate(TO_POOL_QUEUE_LEN, sizeof(network_msg_t));
    ipc_->to_main_q = xQueueCreate(TO_MAIN_QUEUE_LEN, sizeof(network_msg_t));
    
    if (!ipc_->to_main_q || !ipc_->to_pool_q) {
        ESP_LOGE(TAG, "Failed to create IPC queue(s)");
        return;
    }

        // spin off a pool_task that handles RS485 communication, datalink layer and
        // network layer
    if (xTaskCreate(&pool_task, "pool_task", POOL_TASK_STACK_SIZE, this->ipc_, 3, NULL) != pdPASS) {
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

        if (poolstate_rx::update_state(&msg, &new_state) == ESP_OK) {

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
    ESP_LOGCONFIG(TAG, "  RS485 rx pin: %u", this->ipc_->config.rs485_pins.rx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 tx pin: %u", this->ipc_->config.rs485_pins.tx_pin);
    ESP_LOGCONFIG(TAG, "  RS485 rts pin: %u", this->ipc_->config.rs485_pins.rts_pin);

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
OpnPool::update_climates(const poolstate_t * const state)
{
    for (auto climate_id : magic_enum::enum_values<climate_id_t>()) {

        OpnPoolClimate * const climate = this->climates_[enum_index(climate_id)];
        if (!climate) continue;

            // temperatures
        auto const water_temp = &state->temps[enum_index(poolstate_temp_typ_t::WATER)];
        if (!water_temp->valid) continue;

        auto const thermo_typ = climate->get_thermo_typ();
        auto const thermo = &state->thermos[enum_index(thermo_typ)];
        if (!thermo->set_point_in_f.valid) continue;
        if (!thermo->heat_src.valid) continue;
        if (!thermo->heat_src.valid) continue;
        if (!thermo->heating.valid) continue;

        auto const current_temp_f = water_temp->value;
        auto const current_temp_c = fahrenheit_to_celsius(current_temp_f);
        auto const target_temp_c = fahrenheit_to_celsius(thermo->set_point_in_f.value);

            // mode
        auto const switch_idx = _thermo_typ_to_pool_circuit_idx(thermo_typ);
        auto const active_circuit = &state->circuits[switch_idx].active;
        if (!active_circuit->valid) continue;

        climate::ClimateMode mode = active_circuit->value
                                  ? climate::CLIMATE_MODE_HEAT
                                  : climate::CLIMATE_MODE_OFF;

            // custom preset
        auto const custom_preset = enum_str(thermo->heat_src.value);

            // action
        climate::ClimateAction action = thermo->heating.value
                                      ? climate::CLIMATE_ACTION_HEATING
                                      : (mode == climate::CLIMATE_MODE_OFF
                                        ? climate::CLIMATE_ACTION_OFF
                                        : climate::CLIMATE_ACTION_IDLE);

        climate->publish_value_if_changed(current_temp_c, target_temp_c, mode, custom_preset, action);
    }
}

void
OpnPool::update_switches(const poolstate_t * const state)
{
    for (auto switch_id : magic_enum::enum_values<switch_id_t>()) {

        OpnPoolSwitch * const sw = this->switches_[enum_index(switch_id)];
        if (!sw) continue;

        auto const circuit = switch_id_to_network_circuit(switch_id);
        auto const active_circuit = &state->circuits[enum_index(circuit)].active;
        if (!active_circuit->valid) continue;

        sw->publish_value_if_changed(active_circuit->value);
    }
}

void
OpnPool::update_analog_sensors(poolstate_t const * const state)
{
    OpnPoolSensor * const air_temp_sensor = this->sensors_[static_cast<uint8_t>(sensor_id_t::AIR_TEMPERATURE)];
    auto const air_temp = state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::AIR)];
    if (air_temp_sensor != nullptr && air_temp.valid) {
        auto air_temp_c = fahrenheit_to_celsius(air_temp.value);
        air_temp_c = std::round(air_temp_c * 10.0f) / 10.0f;
        air_temp_sensor->publish_value_if_changed(air_temp_c);    
    }

    OpnPoolSensor * const water_temperature_sensor = this->sensors_[static_cast<uint8_t>(sensor_id_t::WATER_TEMPERATURE)];
    auto const water_temp = state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)];
    if (water_temperature_sensor != nullptr && water_temp.valid) {    
        auto water_temp_c = fahrenheit_to_celsius(water_temp.value);
        water_temp_c = std::round(water_temp_c * 10.0f) / 10.0f;
        water_temperature_sensor->publish_value_if_changed(water_temp_c);
    }   
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_POWER)],        
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].power
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_FLOW)],         
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].flow
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_SPEED)],        
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].speed
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::PUMP_ERROR)],        
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].error
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::CHLORINATOR_LEVEL)], 
        state->chlor.level
    );
    _publish_if(
        this->sensors_[static_cast<uint8_t>(sensor_id_t::CHLORINATOR_SALT)],  
        state->chlor.salt
    );
}

void 
OpnPool::update_binary_sensors(poolstate_t const * const state)
{
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::PUMP_RUNNING)],           
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].running
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_SERVICE)],           
        state->modes[static_cast<uint8_t>(network_pool_mode_bits_t::SERVICE)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_TEMPERATURE_INC)],   
        state->modes[static_cast<uint8_t>(network_pool_mode_bits_t::TEMP_INC)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_FREEZE_PROTECTION)], 
        state->modes[static_cast<uint8_t>(network_pool_mode_bits_t::FREEZE_PROT)]
    );
    _publish_if(
        this->binary_sensors_[static_cast<uint8_t>(binary_sensor_id_t::MODE_TIMEOUT)],           
        state->modes[static_cast<uint8_t>(network_pool_mode_bits_t::TIMEOUT)]
    );
}

void 
OpnPool::update_text_sensors(poolstate_t const * const state)
{
    _publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::POOL_SCHED)],
        &state->scheds[static_cast<uint8_t>(network_pool_circuit_t::POOL)]
    );
    _publish_schedule_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::SPA_SCHED)],
        &state->scheds[static_cast<uint8_t>(network_pool_circuit_t::SPA)]
    );
    _publish_enum_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::PUMP_MODE)], 
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].mode
    );    
    _publish_enum_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::PUMP_STATE)],
        state->pumps[enum_index(network_msg_dev_id_t::PRIMARY)].state
    );
    _publish_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CHLORINATOR_NAME)], 
        state->chlor.name
    );
    _publish_enum_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CHLORINATOR_STATUS)],
        state->chlor.status
    );
    _publish_date_and_time_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::SYSTEM_TIME)],
        &state->system.tod
    );
    _publish_version_if(
        this->text_sensors_[static_cast<uint8_t>(text_sensor_id_t::CONTROLLER_FIRMWARE)],
        &state->system.version
    );
}

    // setters

void 
OpnPool::set_rs485_pins(uint8_t const rx_pin, uint8_t const tx_pin, uint8_t const rts_pin)
{
    rs485_pins_.rx_pin = rx_pin;
    rs485_pins_.tx_pin = tx_pin;
    rs485_pins_.rts_pin = rts_pin;
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