/**
 * @file poolstate_rx.cpp
 * @brief Implements pool state update logic for OPNpool ESPHome integration.
 *
 * @details
 * This file implements the core logic for updating and maintaining the pool system state
 * in the OPNpool component. The pool state update layer acts as the bridge between low-level
 * protocol/network messages and the high-level software model of the pool controller and its
 * peripherals (pump, chlorinator, circuits, sensors, etc.).
 *
 * Each supported message type is processed by a dedicated handler function, ensuring modular,
 * robust, and maintainable state updates. The updated state is used for publishing sensor values,
 * driving automation, and integrating with ESPHome and Home Assistant entities.
 *
 * Verbose debug logging and diagnostics are supported via cJSON objects, allowing detailed
 * inspection of state changes and message processing for troubleshooting and development.
 *
 * Design notes:
 * - Assumes a single-threaded environment (as provided by ESPHome); no explicit thread
 *   safety.
 * - Closely coupled with pool_state.h, network_msg.h, and poolstate_rx_log.h for data
 *   structures and logging.
 * - Intended as the main entry point for protocol-driven state updates in the OPNpool
 *   component.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright 2014, 2019, 2022, 2026, Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esphome/core/log.h>
#include <cJSON.h>
#include <type_traits>

#include "to_str.h"
#include "enum_helpers.h"
#include "network.h"
#include "network_msg.h"
#include "ipc.h"
#include "poolstate.h"
#include "opnpool.h"
#include "poolstate_rx_log.h"
#include "opnpool_ids.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

namespace esphome {
namespace opnpool {

namespace poolstate_rx {

static char const * const TAG = "poolstate_rx";

inline network_heat_src_t 
_get_pool_heat_src(uint8_t const combined_heat_src)
{
    return static_cast<network_heat_src_t>((combined_heat_src >> 0) & 0x03);
}

inline network_heat_src_t 
_get_spa_heat_src(uint8_t const combined_heat_src)
{
    return static_cast<network_heat_src_t>((combined_heat_src >> 2) & 0x03);
}

inline bool
_get_pool_heating_status(uint8_t const combined_heat_status)
{
    return (combined_heat_status & 0x04) != 0;  // bit2 is for POOL
}

inline bool
_get_spa_heating_status(uint8_t const combined_heat_status)
{
    return (combined_heat_status & 0x08) != 0;  // bit3 is for SPA
}

inline poolstate_chlor_status_typ_t
_get_chlor_status_from_error(uint8_t const error)
{
    if (error & 0x01) return poolstate_chlor_status_typ_t::LOW_FLOW;
    if (error & 0x02) return poolstate_chlor_status_typ_t::LOW_SALT;
    if (error & 0x04) return poolstate_chlor_status_typ_t::HIGH_SALT;
    if (error & 0x10) return poolstate_chlor_status_typ_t::CLEAN_CELL;
    if (error & 0x40) return poolstate_chlor_status_typ_t::COLD;
    if (error & 0x80) return poolstate_chlor_status_typ_t::OK;
    return poolstate_chlor_status_typ_t::OTHER;
}

/**
 * @brief       Process a controller time message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_time_t message.
 * @param state Pointer to the poolstate_t structure to update.
 * 
 * This function updates the system time and date in the pool state based on the
 * received controller time message. If verbose logging is enabled, the updated
 * time-of-day is added to the debug JSON object.
 */
static void
_ctrl_time(cJSON * const dbg, network_msg_ctrl_time_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    state->system.tod = {
        .date = {
            .valid = true,
            .day = msg->day,
            .month = msg->month,
            .year = static_cast<uint16_t>(2000 + msg->year)
        },
        .time = {
            .valid = true,
            .hour = msg->hour,
            .minute = msg->minute
        }
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_time_and_date(dbg, "tod", &state->system.tod);
        ESP_LOGVV(TAG, "Time-of-day updated: %02u:%02u %02u/%02u/%04u", state->system.tod.time.hour, state->system.tod.time.minute, state->system.tod.date.day, state->system.tod.date.month, state->system.tod.date.year);
    }
}

/**
 * @brief       Process a controller heat response message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_heat_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the pool and spa thermostat values (temperature, set point, heat
 * source) in the pool state based on the received controller heat response message. If
 * verbose logging is enabled, the updated thermostat information is added to the debug
 * JSON object.
 */
static void
_ctrl_heat_resp(cJSON * const dbg, network_msg_ctrl_heat_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    uint8_t const pool_idx = enum_index(poolstate_thermo_typ_t::POOL);
    uint8_t const  spa_idx = enum_index(poolstate_thermo_typ_t::SPA);
    static_assert(pool_idx < enum_count<poolstate_thermo_typ_t>(), "size mismatch for pool_idx");
    static_assert( spa_idx < enum_count<poolstate_thermo_typ_t>(), "size mismatch for spa_idx");

    poolstate_thermo_t * const pool_thermo = &state->thermos[pool_idx];
    poolstate_thermo_t * const spa_thermo  = &state->thermos[spa_idx];

    pool_thermo->valid = true;
    pool_thermo->temp_in_f =  msg->pool_temp;
    pool_thermo->set_point_in_f =  msg->pool_set_point;
    pool_thermo->heat_src = _get_pool_heat_src(msg->combined_heat_src);

    spa_thermo->valid = true;
    spa_thermo->temp_in_f =  msg->spa_temp;
    spa_thermo->set_point_in_f =  msg->spa_set_point;
    spa_thermo->heat_src = _get_spa_heat_src(msg->combined_heat_src);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_thermos(dbg, "thermos", state->thermos, true, true, true, false);
        ESP_LOGVV(TAG, "Thermostat status updated: pool_temp=%u, spa_temp=%u, pool_setpoint=%u, spa_setpoint=%u, pool_heat_src=%u, spa_heat_src=%u", 
            pool_thermo->temp_in_f, spa_thermo->temp_in_f, pool_thermo->set_point_in_f, 
            spa_thermo->set_point_in_f, pool_thermo->heat_src, spa_thermo->heat_src);
    }
}

/**
 * @brief       Process a controller heat set message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_heat_set_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the set point and heat source for the pool and spa thermostats
 * in the pool state based on the received controller heat set message. If verbose logging
 * is enabled, the updated thermostat information is added to the debug JSON object.
 */
static void
_ctrl_heat_set(cJSON * const dbg, network_msg_ctrl_heat_set_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }
    uint8_t const pool_idx = enum_index(poolstate_thermo_typ_t::POOL);
    uint8_t const spa_idx  = enum_index(poolstate_thermo_typ_t::SPA);
    static_assert(pool_idx < enum_count<poolstate_thermo_typ_t>(), "size mismatch for pool_idx");
    static_assert( spa_idx < enum_count<poolstate_thermo_typ_t>(), "size mismatch for spa_idx");

    poolstate_thermo_t * const pool_thermo = &state->thermos[pool_idx];
    poolstate_thermo_t * const spa_thermo  = &state->thermos[spa_idx];
    
    pool_thermo->valid = true;
    pool_thermo->set_point_in_f = msg->pool_set_point;
    pool_thermo->heat_src = _get_pool_heat_src(msg->combined_heat_src);

    spa_thermo->valid = true;
    spa_thermo->set_point_in_f = msg->spa_set_point;
    spa_thermo->heat_src = _get_spa_heat_src(msg->combined_heat_src);
    
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_thermos(dbg, "thermos", state->thermos, false, true, true, false);
        ESP_LOGVV(TAG, "Thermostat set updated: pool_setpoint=%u, spa_setpoint=%u, pool_heat_src=%u, spa_heat_src=%u", 
            pool_thermo->set_point_in_f, spa_thermo->set_point_in_f, 
            pool_thermo->heat_src, spa_thermo->heat_src);
    }
}


static void 
_update_circuit_active_from_bits(poolstate_circuit_t * const arr, uint16_t const bits, uint8_t const count)
{
    for (uint16_t ii = 0, mask = 0x0001; ii < count; ++ii, mask <<= 1) {
        arr[ii].active = {
            .valid = true,
            .value = (bits & mask) != 0
        };
        ESP_LOGVV(TAG, "  arr[%u] = %u", ii, arr[ii].active.value);
    }
}

static void 
_update_circuit_delay_from_bits(poolstate_circuit_t * const arr, uint16_t const bits, uint8_t const count)
{
    for (uint16_t ii = 0, mask = 0x0001; ii < count; ++ii, mask <<= 1) {
        arr[ii].delay = {
            .valid = true,
            .value = (bits & mask) != 0
        };
        ESP_LOGVV(TAG, "  arr[%u] = %u", ii, arr[ii].delay.value);
    }
}

static void 
_update_modes_from_bits(poolstate_bool_t * modes, uint16_t const bits, uint8_t const count)
{
    for (uint16_t ii = 0, mask = 0x0001; ii < count; ++ii, mask <<= 1) {
        modes[ii] = {
            .valid = true,
            .value = (bits & mask) != 0
        };
        ESP_LOGVV(TAG, "  modes[%u] = %u", ii, modes[ii].value);
    }
}

/**
 * @brief             Optionally log raw hex bytes from a controller message.
 *
 * @param dbg         Optional JSON object for verbose debug logging.
 * @param bytes       Pointer to the array of bytes received from the controller.
 * @param state       Pointer to the poolstate_t structure to update.
 * @param no_of_bytes Number of bytes in the array.
 *
 * This function logs the received bytes in hexadecimal format to the debug JSON object
 * if verbose logging is enabled. It does not modify the pool state.
 */
static void
_ctrl_hex_bytes(cJSON * const dbg,  uint8_t const * const bytes, poolstate_t * const state, uint8_t no_of_bytes)
{
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON * const array = cJSON_CreateArray();
        
        for (uint_least8_t ii = 0; ii < no_of_bytes; ii++) {
            char hex_str[3];  // "XX\0"
            ESP_LOGVV(TAG, "byte[%u] = 0x%02X", ii, bytes[ii]);
            snprintf(hex_str, sizeof(hex_str), "%02X", bytes[ii]);
            cJSON_AddItemToArray(array, cJSON_CreateString(hex_str));
        }
        cJSON_AddItemToObject(dbg, "raw", array);
    }
}

/**
 * @brief       Process a controller circuit set message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_circuit_set_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the active state of a specific pool circuit based on the
 * received controller circuit set message. If verbose logging is enabled, the updated
 * circuit value is added to the debug JSON object.
 */
static void
_ctrl_circuit_set(cJSON * const dbg, network_msg_ctrl_circuit_set_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }
    if (msg->circuit_plus_1 == 0) {
        ESP_LOGW(TAG, "circuit_plus_1 == 0");
        return;
    }

    uint8_t const circuit_idx = msg->circuit_plus_1 - 1;
    state->circuits[circuit_idx].active = {
        .valid = true,
        .value = static_cast<bool>(msg->value)
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        network_pool_circuit_t const circuit = static_cast<network_pool_circuit_t>(circuit_idx);

        if (circuit_idx < enum_count<network_pool_circuit_t>()) {
            cJSON_AddNumberToObject(dbg, enum_str(circuit), msg->value);
        } else {
            ESP_LOGW(TAG, "circuit %u>=%u", circuit_idx, enum_count<network_pool_circuit_t>());
        }
        ESP_LOGVV(TAG, "Circuit %u set to %u", circuit_idx, msg->value);
    }
}

/**
 * @brief       Process a controller schedule response message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_sched_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the pool state with schedule information for each circuit based on
 * the received controller schedule response message. If verbose logging is enabled, the
 * updated schedule information is added to the debug JSON object.
 */
static void
_ctrl_sched_resp(cJSON * const dbg, network_msg_ctrl_sched_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }
    poolstate_sched_t * state_scheds = state->scheds;    
    memset(state_scheds, 0, sizeof(state->scheds));

    for (const auto& sched : msg->scheds) {

        if (sched.circuit_plus_1 == 0) {
            // nothing wrong with this. the schedule entry is just unused
            continue;
        }

        network_pool_circuit_t const circuit =
            static_cast<network_pool_circuit_t>(sched.circuit_plus_1 - 1);
        uint8_t const circuit_idx = enum_index(circuit);

        uint16_t const startHi = sched.prg_start_hi;
        uint16_t const stopHi  = sched.prg_stop_hi;
        uint16_t const start = (startHi << 8) | sched.prg_start_lo;
        uint16_t const stop  = ( stopHi << 8) | sched.prg_stop_lo;

        if (circuit_idx < std::size(state->scheds)) {
            state_scheds[circuit_idx] = {
                .valid = true,
                .active = true,
                .start = start,
                .stop = stop
            };
            ESP_LOGVV(TAG, "Schedule updated for %s: start=%u, stop=%u", enum_str(circuit), start, stop);
        } else {
            ESP_LOGW(TAG, "circuit %u>=%zu", circuit_idx, std::size(state->scheds));
        }
    }

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_sched(dbg, "scheds", state->scheds, true);
    }
}

static void
_update_circuits(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_circuit_t * const circuits)
{
    constexpr uint8_t pool_idx = enum_index(network_pool_circuit_t::POOL);
    constexpr uint8_t spa_idx  = enum_index(network_pool_circuit_t::SPA);

        // update circuits[].active
    uint16_t const bitmask_active_circuits = ((uint16_t)msg->active_hi << 8) | msg->active_lo;
    static_assert(enum_count<network_pool_mode_bits_t>() <= enum_count<network_pool_circuit_t>(), "size mismatch for circuits");
    _update_circuit_active_from_bits(circuits, bitmask_active_circuits, enum_count<network_pool_circuit_t>());

        // if both SPA and POOL bits are set, only SPA runs
    if (circuits[spa_idx].active.value) {
        circuits[pool_idx].active.value = false;
    }

        // update circuits[].delay
    uint8_t const bitmask_delay_circuits = msg->delay;
    static_assert(enum_count<network_pool_mode_bits_t>() <= enum_count<network_pool_circuit_t>(), "size mismatch for circuits");
    _update_circuit_delay_from_bits(circuits, bitmask_delay_circuits, enum_count<network_pool_circuit_t>());

}

static void
_update_thermos(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_thermo_t * const thermos, poolstate_circuit_t const * const circuits)
{
        // update circuits.thermos (only update when the pump is running)
    constexpr uint8_t pool_therm_idx = enum_index(poolstate_thermo_typ_t::POOL);
    constexpr uint8_t spa_therm_idx  = enum_index(poolstate_thermo_typ_t::SPA);
    static_assert(pool_therm_idx < enum_count<poolstate_thermo_typ_t>(), "pool_therm_idx OOB");
    static_assert(spa_therm_idx < enum_count<poolstate_thermo_typ_t>(), "spa_therm_idx OOB");
    poolstate_thermo_t * const pool_thermo = &thermos[pool_therm_idx];
    poolstate_thermo_t * const spa_thermo  = &thermos[spa_therm_idx];

    constexpr uint8_t pool_circuit_idx = enum_index(network_pool_circuit_t::POOL);
    constexpr uint8_t spa_circuit_idx  = enum_index(network_pool_circuit_t::SPA);
    poolstate_bool_t const * const pool_circuit = &circuits[pool_circuit_idx].active;
    poolstate_bool_t const * const spa_circuit  = &circuits[spa_circuit_idx].active;

        // this leaves a gap where the pump is not running, so we still update the temperature
    if (spa_circuit->valid && spa_circuit->value) {
        spa_thermo->temp_in_f = msg->pool_temp;
    }
    if (pool_circuit->valid && pool_circuit->value) {
        pool_thermo->temp_in_f = msg->pool_temp;
    }
    pool_thermo->valid    = true;
    pool_thermo->heating  = _get_pool_heating_status(msg->combined_heat_status);
    pool_thermo->heat_src = _get_pool_heat_src(msg->combined_heat_srcs);

    spa_thermo->valid    = true;
    spa_thermo->heating  = _get_spa_heating_status(msg->combined_heat_status);
    spa_thermo->heat_src = _get_spa_heat_src(msg->combined_heat_srcs);
}

static void
_update_modes(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_bool_t * const modes)
{
    static_assert(enum_count<network_pool_mode_bits_t>() <= enum_count<network_pool_mode_bits_t>(), "size err for modes->is_set");

    _update_modes_from_bits(modes, msg->mode_bits, enum_count<network_pool_mode_bits_t>());
}

static void
_update_system_time(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_time_t * const time)
{
    *time = {
        .valid = true,
        .hour = msg->hour,
        .minute = msg->minute
    };
    // PS date is updated through `network_msg_ctrl_time`
}

static void
_update_temps(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_uint8_t * const temps)
{
    uint8_t const air_idx = enum_index(poolstate_temp_typ_t::AIR);
    uint8_t const water_idx = enum_index(poolstate_temp_typ_t::WATER);
    static_assert(air_idx < enum_count<poolstate_temp_typ_t>(), "size err for air_idx");
    static_assert(water_idx < enum_count<poolstate_temp_typ_t>(), "size err for water_idx");

    temps[air_idx] = {
        .valid = true,
        .value = msg->air_temp
    };
    temps[water_idx] = {
        .valid = true,
        .value = msg->water_temp
    };
}

/**
 * @brief       Process a controller state broadcast message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_state_bcast_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the entire pool state (circuits, modes, thermostats, system time,
 * and temperatures) based on the received controller state broadcast message. If verbose
 * logging is enabled, the updated state is added to the debug JSON object.
 */
static void
_ctrl_state(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg,  poolstate_t * state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    _update_circuits(dbg, msg, state->circuits);
    _update_thermos(dbg, msg, state->thermos, state->circuits); 
    _update_modes(dbg, msg, state->modes);
    _update_system_time(dbg, msg, &state->system.tod.time);
    _update_temps(dbg, msg, state->temps);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_state(dbg, "state", state);
        ESP_LOGVV(TAG, "State updated from CTRL_STATE message");
    }
}

/**
 * @brief       Process a controller version response message and update the pool state.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_version_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the firmware version in the pool state based on the received
 * controller version response message. If verbose logging is enabled, the updated
 * version information is added to the debug JSON object.
 */
static void
_ctrl_version_resp(cJSON * const dbg, network_msg_ctrl_version_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    state->system.version = {
        .valid = true,
        .major = msg->major,
        .minor = msg->minor
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_version(dbg, "firmware", &state->system.version);
        ESP_LOGVV(TAG, "Firmware version updated to %u.%u", state->system.version.major, state->system.version.minor);
    }
}

/**
 * @brief       Process a pump register set message and log the register update.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_reg_set_t message.
 *
 * This function decodes the pump register address and value from the message and, if
 * verbose logging is enabled, logs the register update to the debug JSON object.
 */
static void
_pump_reg_set(cJSON * const dbg, network_msg_pump_reg_set_t const * const msg)
{
    if (!msg) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    // no change to poolstate

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        uint16_t const address = (msg->address_hi << 8) | msg->address_lo;
        uint16_t const value = (msg->value_hi << 8) | msg->value_lo;
        network_pump_program_addr_t const address_enum =
            static_cast<network_pump_program_addr_t>(address);
        poolstate_rx_log::add_pump_program(dbg, network_pump_program_addr_str(address_enum), value);
    }
}


/**
 * @brief       Process a pump register set response message and log the register value.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_reg_resp_t message.
 *
 * This function decodes the register value from the message and, if verbose logging is
 * enabled, logs the value to the debug JSON object.
 */
static void
_pump_reg_set_resp(cJSON * const dbg, network_msg_pump_reg_resp_t const * const msg)
{
    if (!msg) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    // no change to poolstate

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        uint16_t const value_hi = msg->value_hi;
        uint16_t const value = (value_hi << 8) | msg->value_lo;
        poolstate_rx_log::add_pump_program(dbg, "resp", value);
    }
}

/**
 * @brief       Process a pump control message and log the control value.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_ctrl_t message.
 *
 * This function logs the pump control value to the debug JSON object if verbose logging
 * is enabled.
 */
static void
_pump_ctrl(cJSON * const dbg, network_msg_pump_ctrl_t const * const msg)
{
    if (!msg) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    // no change to poolstate

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
       poolstate_rx_log::add_pump_ctrl(dbg, "ctrl", msg->ctrl);
    }
}

/**
 * @brief      Process a pump mode message, update the pool state, and log the mode.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_mode_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the pump mode in the pool state and logs the mode to the debug
 * JSON object if verbose logging is enabled.
 */
static void
_pump_mode(cJSON * const dbg, network_msg_pump_mode_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    state->pump.mode = {
        .valid = true,
        .value = static_cast<network_pump_mode_t>(msg->mode)
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_pump_mode(dbg, "mode", state->pump.mode.value);
        ESP_LOGVV(TAG, "Pump mode updated to %s", enum_str(state->pump.mode.value));
    }
}

/**
 * @brief      Process a pump run message, update the running state, and log the status.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_run_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the running state of the pump in the pool state and logs the
 * status to the debug JSON object if verbose logging is enabled.
 */
static void
_pump_run(cJSON * const dbg,
    network_msg_pump_run_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    bool const running     = msg->running == network_pump_running_t::ON;
    bool const not_running = msg->running == network_pump_running_t::OFF;
    if (!running && !not_running) {
        ESP_LOGW(TAG, "running state err 0x%02X in %s", static_cast<uint8_t>(msg->running), __func__);
        return;
    }    

    state->pump.running = {
        .valid = true,
        .value = running
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {   
        poolstate_rx_log::add_pump_running(dbg, "running", state->pump.running.value);
        ESP_LOGVV(TAG, "Pump running state updated to %u", state->pump.running.value);
    }
}

/**
 * @brief       Process a pump status response message, update the pool state, and log the status.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_pump_status_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates all pump status fields in the pool state and logs the status to
 * the debug JSON object if verbose logging is enabled.
 */
static void
_pump_status(cJSON * const dbg, network_msg_pump_status_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    bool const running     = msg->running == network_pump_running_t::ON;
    bool const not_running = msg->running == network_pump_running_t::OFF;

    if (!running && !not_running) {
        ESP_LOGW(TAG, "running state err 0x%02X (%u %u) in %s", static_cast<uint8_t>(msg->running), running, not_running, __func__);
        return;
    }

    uint16_t const power = ((uint16_t)msg->power_hi << 8) | msg->power_lo;
    uint16_t const speed = ((uint16_t)msg->speed_hi << 8) | msg->speed_lo;

    state->pump = {
        .time = {
            .valid  = true,
            .hour   = msg->clock_hr,
            .minute = msg->clock_min
        },
        .mode = {
            .valid = true,
            .value = static_cast<network_pump_mode_t>(msg->mode)
        },
        .running = {
            .valid = true,
            .value = running
        },
        .state   = {
            .valid = true,
            .value = static_cast<network_pump_state_t>(msg->state)
        },
        .power   = {
            .valid = true,
            .value = power
        },
        .flow    = {
            .valid = true,
            .value = msg->flow
        },
        .speed   = {
            .valid = true,
            .value = speed
        },
        .level   = {
            .valid = true,      
            .value = msg->level
        },
        .error   = {
            .valid = true,
            .value = msg->error
        },
        .timer   = {
            .valid = true,
            .value = msg->remaining_min
        }
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_pump(dbg, "status", state);
        ESP_LOGVV(TAG, "Pump status updated: running=%d, mode=%s, state=%s, power=%u, speed=%u, flow=%u, level=%u, error=%u, timer=%u, time=%02u:%02u", 
            state->pump.running.value, enum_str(state->pump.mode.value), enum_str(state->pump.state.value),
            state->pump.power.value, state->pump.speed.value, state->pump.flow.value, state->pump.level.value,
            state->pump.error.value, state->pump.timer.value, state->pump.time.hour, state->pump.time.minute);
    }
}

/**
 * @brief       Process a controller set acknowledgment message and log the result.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_ctrl_set_ack_t message.
 *
 * This function logs the acknowledgment type to the debug JSON object if verbose logging
 * is enabled.
 */
static void
_ctrl_set_ack(cJSON * const dbg, network_msg_ctrl_set_ack_t const * const msg)
{
    if (!msg) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    // no change to poolstate
    //     2BD we could.., e.g. when a circuit is changed ..

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON_AddStringToObject(dbg, "ack", enum_str(msg->typ));
    }
}

/**
 * @brief       Process a chlorine generator name response message, update the pool state, and log the status.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_chlor_name_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the chlorine generator name and salt level in the pool state and
 * logs the status to the debug JSON object if verbose logging is enabled.
 */
static void
_chlor_name_resp(cJSON * const dbg, network_msg_chlor_name_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    state->chlor.salt = {
        .valid = true,
        .value = static_cast<uint16_t>((uint16_t)msg->salt * 50)
    };

    size_t name_size = sizeof(state->chlor.name.value);
    strncpy(state->chlor.name.value, msg->name, name_size);
    state->chlor.name.value[name_size - 1] = '\0';
    state->chlor.name.valid = true;

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON_AddNumberToObject(dbg, "salt", state->chlor.salt.value);
        cJSON_AddStringToObject(dbg, "name", state->chlor.name.value);
        ESP_LOGV(TAG, "Chlorine status updated: salt=%u, name=%s", state->chlor.salt.value, state->chlor.name.value);
    }
}

/**
 * @brief       Process a chlorine generator level set message, update the pool state, and log the status.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_chlor_level_set_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the chlorine generator level in the pool state and logs the
 * status to the debug JSON object if verbose logging is enabled.
 */
static void
_chlor_level_set(cJSON * const dbg, network_msg_chlor_level_set_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }   

    state->chlor.level = {
        .valid = true,
        .value = msg->level
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON_AddNumberToObject(dbg, "level", state->chlor.level.value);
        ESP_LOGVV(TAG, "Chlorine level updated: level=%u", state->chlor.level.value);
    }
}

/**
 * @brief       Process a chlorine generator level set response message, update the pool state, and log the status.
 *
 * @param dbg   Optional JSON object for verbose debug logging.
 * @param msg   Pointer to the received network_msg_chlor_level_resp_t message.
 * @param state Pointer to the poolstate_t structure to update.
 *
 * This function updates the chlorine generator salt level and status in the pool state and logs the status to
 * the debug JSON object if verbose logging is enabled.
 * Note: good salt range is 2600 to 4500 ppm.
 */
static void
_chlor_level_set_resp(cJSON * const dbg, network_msg_chlor_level_resp_t const * const msg, poolstate_t * const state)
{
    if (!msg || !state) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return;
    }

    state->chlor.salt = {
        .valid = true,
        .value = static_cast<uint16_t>((uint16_t)msg->salt * 50)
    };
    state->chlor.status = {
        .valid = true,
        .value = _get_chlor_status_from_error(msg->error)
    };  

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        poolstate_rx_log::add_chlor_resp(dbg, "chlor", &state->chlor);
        ESP_LOGVV(TAG, "Chlorine status updated: salt=%u, status=%s", state->chlor.salt.value, enum_str(state->chlor.status.value));
    }
}

/**
 * @brief           Process a received network message and update the pool state.
 *
 * @param msg       Pointer to the received network_msg_t message (must not be null).
 * @param new_state Pointer to the poolstate_t structure to update (must not be null).
 * @return          ESP_OK if the state was updated and processed successfully, ESP_FAIL otherwise.
 *
 * This function dispatches the received network message to the appropriate handler,
 * updating the provided pool state structure based on the message contents. It also
 * logs detailed debug information to a cJSON object if verbose logging is enabled.
 *
 * The function sets the 'valid' flag in the state, updates all relevant fields according
 * to the message type, and optionally logs the update as JSON. It is the main entry point
 * for state updates in response to protocol messages.
 */
esp_err_t
update_state(network_msg_t const * const msg, poolstate_t * const new_state)
{
    if (msg == nullptr || new_state == nullptr) {
        ESP_LOGW(TAG, "null to %s", __func__);
        return ESP_FAIL;
    }

        // adjust the new_state based on the incoming message
    cJSON * const dbg = cJSON_CreateObject();

    switch (msg->typ) {
        case network_msg_typ_t::CTRL_SET_ACK:  // response to various set requests
            _ctrl_set_ack(dbg, &msg->u.ctrl_set_ack);
            break;
        case network_msg_typ_t::CTRL_CIRCUIT_SET:
            _ctrl_circuit_set(dbg, &msg->u.ctrl_circuit_set, new_state);
            break;
        case network_msg_typ_t::CTRL_SCHED_REQ:
            break;
        case network_msg_typ_t::CTRL_SCHED_RESP:
            _ctrl_sched_resp(dbg, &msg->u.ctrl_sched_resp, new_state);
            break;
        case network_msg_typ_t::CTRL_STATE_BCAST:
            _ctrl_state(dbg, &msg->u.ctrl_state, new_state);
            break;
        case network_msg_typ_t::CTRL_TIME_REQ:
            break;
        case network_msg_typ_t::CTRL_TIME_RESP:
            _ctrl_time(dbg, &msg->u.ctrl_time_resp, new_state);
            break;
        case network_msg_typ_t::CTRL_TIME_SET:
            _ctrl_time(dbg, &msg->u.ctrl_time_set, new_state);
            break;
        case network_msg_typ_t::CTRL_HEAT_REQ:
            break;
        case network_msg_typ_t::CTRL_HEAT_RESP:
            _ctrl_heat_resp(dbg, &msg->u.ctrl_heat_resp, new_state);
            break;
        case network_msg_typ_t::CTRL_HEAT_SET:
            _ctrl_heat_set(dbg, &msg->u.ctrl_heat_set, new_state);
            break;
        case network_msg_typ_t::CTRL_LAYOUT_REQ:
        case network_msg_typ_t::CTRL_LAYOUT_RESP:
        case network_msg_typ_t::CTRL_LAYOUT_SET:
        case network_msg_typ_t::CTRL_VALVE_REQ:
        case network_msg_typ_t::CTRL_SOLARPUMP_REQ:
        case network_msg_typ_t::CTRL_DELAY_REQ:
        case network_msg_typ_t::CTRL_HEAT_SETPT_REQ:
        case network_msg_typ_t::CTRL_VERSION_REQ:
            break;
        case network_msg_typ_t::CTRL_SCHEDS_REQ:
        case network_msg_typ_t::CTRL_CIRC_NAMES_REQ:
        case network_msg_typ_t::CTRL_CHEM_REQ:
        case network_msg_typ_t::CHLOR_NAME_REQ:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, 1);
            break;
        case network_msg_typ_t::CTRL_VERSION_RESP:
            _ctrl_version_resp(dbg, &msg->u.ctrl_version_resp, new_state);
            break;
        case network_msg_typ_t::CTRL_VALVE_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_valve_resp_t));
            break;
        case network_msg_typ_t::CTRL_SOLARPUMP_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_solarpump_resp_t));
            break;
        case network_msg_typ_t::CTRL_DELAY_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_delay_resp_t));
            break;
        case network_msg_typ_t::CTRL_HEAT_SETPT_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_heat_setpt_resp_t));
            break;
        case network_msg_typ_t::CTRL_CIRC_NAMES_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_circ_names_resp_t));
            break;
        case network_msg_typ_t::CTRL_SCHEDS_RESP:
            _ctrl_hex_bytes(dbg, msg->u.bytes, new_state, sizeof(network_msg_ctrl_scheds_resp_t));
            break;
        case network_msg_typ_t::PUMP_REG_SET:
            _pump_reg_set(dbg, &msg->u.pump_reg_set);
            break;
        case network_msg_typ_t::PUMP_REG_RESP:
            _pump_reg_set_resp(dbg, &msg->u.pump_reg_set_resp);
            break;
        case network_msg_typ_t::PUMP_CTRL_SET:
        case network_msg_typ_t::PUMP_CTRL_RESP:
            _pump_ctrl(dbg, &msg->u.pump_ctrl);
            break;
        case network_msg_typ_t::PUMP_MODE_SET:
        case network_msg_typ_t::PUMP_MODE_RESP:
            _pump_mode(dbg, &msg->u.pump_mode, new_state);
            break;
        case network_msg_typ_t::PUMP_RUN_SET:
        case network_msg_typ_t::PUMP_RUN_RESP:
            _pump_run(dbg, &msg->u.pump_run, new_state);
            break;
        case network_msg_typ_t::PUMP_STATUS_REQ:
             break;
        case network_msg_typ_t::PUMP_STATUS_RESP:
            _pump_status(dbg, &msg->u.pump_status_resp, new_state);
            break;
        case network_msg_typ_t::CHLOR_PING_REQ:
        case network_msg_typ_t::CHLOR_PING_RESP:
            break;
        case network_msg_typ_t::CHLOR_NAME_RESP:
            _chlor_name_resp(dbg, &msg->u.chlor_name_resp, new_state);
            break;
        case network_msg_typ_t::CHLOR_LEVEL_SET:
            _chlor_level_set(dbg, &msg->u.chlor_level_set, new_state);
            break;
        case network_msg_typ_t::CHLOR_LEVEL_RESP:
            _chlor_level_set_resp(dbg, &msg->u.chlor_level_resp, new_state);
            break;
        default:
            ESP_LOGW(TAG, "Received unknown message type: %u", static_cast<uint8_t>(msg->typ));
            break;
    }

    bool const frequent = // msg->typ == network_msg_typ_t::CTRL_STATE_BCAST ||
                          msg->typ == network_msg_typ_t::CHLOR_LEVEL_SET ||
                          msg->typ == network_msg_typ_t::PUMP_CTRL_SET ||
                          msg->typ == network_msg_typ_t::PUMP_CTRL_RESP ||
                          msg->typ == network_msg_typ_t::PUMP_RUN_SET ||
                          msg->typ == network_msg_typ_t::PUMP_RUN_RESP ||
                          msg->typ == network_msg_typ_t::PUMP_STATUS_REQ ||
                          msg->typ == network_msg_typ_t::PUMP_STATUS_RESP;

    bool const verbose = ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE;
    bool const very_verbose = ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE;

    if ((verbose && !frequent) || very_verbose) {
        size_t const json_size = 1024;
        char * const json = static_cast<char *>(calloc(1, json_size));
        if (json == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate memory for JSON string");
            cJSON_Delete(dbg);
            return ESP_FAIL;
        }
        bool print_success = cJSON_PrintPreallocated(dbg, json, json_size, false);
        if (!print_success) {
            ESP_LOGE(TAG, "Failed to print JSON string");
            free(json);
            cJSON_Delete(dbg);
            return ESP_FAIL;
        }
        ESP_LOGV(TAG, "{%s: %s}\n", enum_str(msg->typ), json);
        free(json);
    }
    cJSON_Delete(dbg);
    return ESP_OK;
}

} // namespace poolstate_rx

} // namespace opnpool
} // namespace esphome  