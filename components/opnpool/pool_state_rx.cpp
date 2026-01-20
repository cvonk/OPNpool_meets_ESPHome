/**
 * @file pool_state_rx.cpp
 * @brief Implements pool state update logic for OPNpool ESPHome integration.
 *
 * @details
 * This file provides the main logic for updating the pool state in response to all
 * supported protocol messages received from the pool controller, pump, and chlorine
 * generator. It contains message dispatch, field extraction, and state mutation logic, as
 * well as integration with Home Assistant via ESPHome. Each message type is handled by a
 * dedicated function, ensuring robust and maintainable state updates. Verbose debug
 * logging is supported using cJSON objects for diagnostics and troubleshooting, and all
 * state changes are optionally logged in detail.
 *
 * Design notes:
 * - Assumes a single-threaded environment (as provided by ESPHome); no explicit thread
 *   safety.
 * - Closely coupled with pool_state.h, network_msg.h, and pool_state_rx_log.h for data
 *   structures and logging.
 * - Intended as the main entry point for protocol-driven state updates in the OPNpool
 *   component.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright 2014, 2019, 2022, 2026, Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <esp_system.h>
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
#include "pool_state.h"
#include "opnpool.h"
#include "pool_state_rx_log.h"

#include <iterator>
#include "opnpool_ids.h"

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

namespace esphome {
namespace opnpool {

namespace pool_state_rx {

static char const * const TAG = "pool_state_rx";

inline network_heat_src_t 
_get_pool_heat_src(uint8_t combined_heat_src)
{
    return static_cast<network_heat_src_t>((combined_heat_src >> 0) & 0x03);
}

inline network_heat_src_t 
_get_spa_heat_src(uint8_t combined_heat_src)
{
    return static_cast<network_heat_src_t>((combined_heat_src >> 2) & 0x03);
}

inline bool
_get_pool_heating_status(uint8_t combined_heat_status)
{
    return (combined_heat_status & 0x04) != 0;  // bit2 is for POOL
}

inline bool
_get_spa_heating_status(uint8_t combined_heat_status)
{
    return (combined_heat_status & 0x08) != 0;  // bit3 is for SPA
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

    state->system.tod.time.minute = msg->minute;
    state->system.tod.time.hour   = msg->hour;
    state->system.tod.date.day    = msg->day;
    state->system.tod.date.month  = msg->month;
    state->system.tod.date.year   = (uint16_t)(2000) + msg->year;

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_time_and_date(dbg, "tod", &state->system.tod);
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
    static_assert(pool_idx < ARRAY_SIZE(state->thermos), "size mismatch for pool_idx");
    static_assert( spa_idx < ARRAY_SIZE(state->thermos), "size mismatch for spa_idx");

    state->thermos[pool_idx].temp_in_f =  msg->pool_temp;
    state->thermos[ spa_idx].temp_in_f =  msg->spa_temp;
    state->thermos[pool_idx].set_point_in_f =  msg->pool_set_point;
    state->thermos[ spa_idx].set_point_in_f =  msg->spa_set_point;
    state->thermos[pool_idx].heat_src = _get_pool_heat_src(msg->combined_heat_src);
    state->thermos[ spa_idx].heat_src = _get_spa_heat_src(msg->combined_heat_src);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_thermos(dbg, "thermos", state->thermos, true, true, true, false);
        ESP_LOGVV(TAG, "Thermostat status updated: pool_temp=%u, spa_temp=%u, pool_setpoint=%u, spa_setpoint=%u, pool_heat_src=%u, spa_heat_src=%u", 
            state->thermos[pool_idx].temp_in_f, state->thermos[spa_idx].temp_in_f, state->thermos[pool_idx].set_point_in_f, 
            state->thermos[spa_idx].set_point_in_f, state->thermos[pool_idx].heat_src, state->thermos[spa_idx].heat_src);
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
    static_assert(pool_idx < ARRAY_SIZE(state->thermos), "size mismatch for pool_idx");
    static_assert( spa_idx < ARRAY_SIZE(state->thermos), "size mismatch for spa_idx");

    state->thermos[pool_idx].set_point_in_f = msg->pool_set_point;
    state->thermos[ spa_idx].set_point_in_f = msg->spa_set_point;
    state->thermos[pool_idx].heat_src = _get_pool_heat_src(msg->combined_heat_src);
    state->thermos[ spa_idx].heat_src = _get_spa_heat_src(msg->combined_heat_src);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_thermos(dbg, "thermos", state->thermos, false, true, true, false);
        ESP_LOGVV(TAG, "Thermostat set updated: pool_setpoint=%u, spa_setpoint=%u, pool_heat_src=%u, spa_heat_src=%u", 
            state->thermos[pool_idx].set_point_in_f, state->thermos[spa_idx].set_point_in_f, 
            state->thermos[pool_idx].heat_src, state->thermos[spa_idx].heat_src);
    }
}


/**
 * @brief      Helper to update a bool array from a bitfield value.
 *
 * @param arr      Pointer to the bool array to update.
 * @param bits     Bitfield value to extract bits from.
 * @param count    Number of bits/array elements to update.
 */
static void 
_update_bool_array_from_bits(bool * const arr, uint16_t const bits, uint8_t const count)
{
    for (uint16_t ii = 0, mask = 0x0001; ii < count; ++ii, mask <<= 1) {
        arr[ii] = (bits & mask) != 0;
        ESP_LOGVV(TAG, "  arr[%u] = %u", ii, arr[ii]);
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
    state->circuits.active[circuit_idx] = msg->value;

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

#if 0    
    network_msg_ctrl_sched_resp_sub_t const * msg_sched = msg->scheds;
    for (uint_least8_t ii = 0; ii < NETWORK_MSG_CTRL_SCHED_COUNT; ii++, msg_sched++) {
#endif
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
            state_scheds[circuit_idx] = (poolstate_sched_t) {
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
        pool_state_rx_log::add_sched(dbg, "scheds", state->scheds, true);
    }
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

        // update state->circuits.active
    uint16_t const bitmask_active_circuits = ((uint16_t)msg->active_hi << 8) | msg->active_lo;
    static_assert(NETWORK_POOL_MODE_BITS_COUNT <= ARRAY_SIZE(state->circuits.active), "size mismatch for state->circuits.active");
    _update_bool_array_from_bits(state->circuits.active, bitmask_active_circuits, enum_count<network_pool_circuit_t>());

        // if both SPA and POOL bits are set, only SPA runs
    if (state->circuits.active[enum_index(network_pool_circuit_t::SPA)]) {
        state->circuits.active[enum_index(network_pool_circuit_t::POOL)] = false;
    }

        // update state->circuits.delay
    uint8_t const bitmask_delay_circuits = msg->delay;
    static_assert(NETWORK_POOL_MODE_BITS_COUNT <= ARRAY_SIZE(state->circuits.delay), "size mismatch for state->circuits.delay");
    _update_bool_array_from_bits(state->circuits.delay, bitmask_delay_circuits, enum_count<network_pool_circuit_t>());

        // update state->circuits.thermos (only update when the pump is running)
    constexpr uint8_t pool_idx = enum_index(poolstate_thermo_typ_t::POOL);
    constexpr uint8_t spa_idx  = enum_index(poolstate_thermo_typ_t::SPA);
    static_assert(pool_idx < ARRAY_SIZE(state->thermos) && spa_idx < ARRAY_SIZE(state->thermos), "pool_idx/spa_idx OOB for state->thermos");

    if (state->circuits.active[enum_index(network_pool_circuit_t::SPA)]) {
        state->thermos[spa_idx].temp_in_f = msg->pool_temp;
    }
    if (state->circuits.active[enum_index(network_pool_circuit_t::POOL)]) {
        state->thermos[pool_idx].temp_in_f = msg->pool_temp;
    }
    state->thermos[pool_idx].heating  = _get_pool_heating_status(msg->combined_heat_status);
    state->thermos[spa_idx ].heating  = _get_spa_heating_status(msg->combined_heat_status);
    state->thermos[pool_idx].heat_src = _get_pool_heat_src(msg->combined_heat_srcs);
    state->thermos[spa_idx ].heat_src = _get_spa_heat_src(msg->combined_heat_srcs);

        // update state->modes.is_set
    uint8_t const bitmask_active_modes = msg->mode_bits;
    static_assert(NETWORK_POOL_MODE_BITS_COUNT <= ARRAY_SIZE(state->modes.is_set), "size err for state->modes.is_set");
    _update_bool_array_from_bits(state->modes.is_set, bitmask_active_modes, NETWORK_POOL_MODE_BITS_COUNT);

        // update state->system (date is updated through `network_msg_ctrl_time`)
    state->system.tod.time.minute = msg->minute;
    state->system.tod.time.hour = msg->hour;

        // update state->temps
    uint8_t const air_idx = enum_index(poolstate_temp_typ_t::AIR);
    uint8_t const water_idx = enum_index(poolstate_temp_typ_t::WATER);
    static_assert(air_idx < ARRAY_SIZE(state->temps), "size err for air_idx");
    static_assert(water_idx < ARRAY_SIZE(state->temps), "size err for water_idx");

    state->temps[air_idx].temp = msg->air_temp;
    state->temps[water_idx].temp = msg->water_temp;

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_state(dbg, "state", state);
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
        .major = msg->major,
        .minor = msg->minor
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_version(dbg, "firmware", &state->system.version);
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

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        uint16_t const address = (msg->address_hi << 8) | msg->address_lo;
        uint16_t const value = (msg->value_hi << 8) | msg->value_lo;
        network_pump_program_addr_t const address_enum =
            static_cast<network_pump_program_addr_t>(address);
        pool_state_rx_log::add_pump_program(dbg, network_pump_program_addr_str(address_enum), value);
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

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        uint16_t const value_hi = msg->value_hi;
        uint16_t const value = (value_hi << 8) | msg->value_lo;
        pool_state_rx_log::add_pump_program(dbg, "resp", value);
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

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
       pool_state_rx_log::add_pump_ctrl(dbg, "ctrl", msg->ctrl);
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

    state->pump.mode = static_cast<network_pump_mode_t>(msg->mode);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_pump_mode(dbg, "mode", state->pump.mode);
        ESP_LOGVV(TAG, "Pump mode updated to %s", enum_str(state->pump.mode));
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
        ESP_LOGW(TAG, "running state err 0x%02X in %s", msg->running, __func__);
        return;
    }    
    state->pump.running = running;

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_pump_running(dbg, "running", state->pump.running);
        ESP_LOGVV(TAG, "Pump running state updated to %u", state->pump.running);
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
        .time    = {
            .hour   = msg->clock_hr,
            .minute = msg->clock_min
        },
        .mode    = static_cast<network_pump_mode_t>(msg->mode),
        .running = running,
        .state   = static_cast<network_pump_state_t>(msg->state),
        .power   = power,
        .flow    = msg->flow,
        .speed   = speed,
        .level   = msg->level,
        .error   = msg->error,
        .timer   = msg->remaining_min
    };

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_pump(dbg, "status", state);
        ESP_LOGVV(TAG, "Pump status updated: running=%d, mode=%s, state=%s, power=%u, speed=%u, flow=%u, level=%u, error=%u, timer=%u, time=%02u:%02u", state->pump.running, enum_str(state->pump.mode), enum_str(state->pump.state), state->pump.power, state->pump.speed, state->pump.flow, state->pump.level, state->pump.error, state->pump.timer, state->pump.time.hour, state->pump.time.minute);
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
    state->chlor.salt = (uint16_t)msg->salt * 50;

    if (msg->name == nullptr) {
        ESP_LOGW(TAG, "null chlorine name");
        state->chlor.name[0] = '\0';
    } else {        
        size_t name_size = sizeof(state->chlor.name);
        strncpy(state->chlor.name, msg->name, name_size);
        state->chlor.name[name_size - 1] = '\0';
    }

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON_AddNumberToObject(dbg, "salt", state->chlor.salt);
        cJSON_AddStringToObject(dbg, "name", state->chlor.name);
        ESP_LOGVV(TAG, "Chlorine status updated: salt=%u, name=%s", state->chlor.salt, state->chlor.name);
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

    state->chlor.level = msg->level;

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        cJSON_AddNumberToObject(dbg, "level", state->chlor.level);
        ESP_LOGVV(TAG, "Chlorine level updated: level=%u", state->chlor.level);
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

    state->chlor.salt = (uint16_t)msg->salt * 50;

    if (msg->error & 0x01) {
        state->chlor.status = poolstate_chlor_status_t::LOW_FLOW;
    } else if (msg->error & 0x02) {
        state->chlor.status = poolstate_chlor_status_t::LOW_SALT;
    } else if (msg->error & 0x04) {
        state->chlor.status = poolstate_chlor_status_t::HIGH_SALT;
    } else if (msg->error & 0x10) {
        state->chlor.status = poolstate_chlor_status_t::CLEAN_CELL;
    } else if (msg->error & 0x40) {
        state->chlor.status = poolstate_chlor_status_t::COLD;
    } else if (msg->error & 0x80) {
        state->chlor.status = poolstate_chlor_status_t::OK;
    } else {
        state->chlor.status = poolstate_chlor_status_t::OTHER;
        ESP_LOGW(TAG, "Unknown error code received in rx_chlor_level_set_resp: 0x%02X", msg->error);
    }

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        pool_state_rx_log::add_chlor_resp(dbg, "chlor", &state->chlor);
        ESP_LOGVV(TAG, "Chlorine status updated: salt=%u, status=%s", state->chlor.salt, enum_str(state->chlor.status));
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

        // the new state will always be valid
    new_state->valid = true;

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

    bool const frequent = msg->typ == network_msg_typ_t::CTRL_STATE_BCAST ||
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

} // namespace pool_state_rx

} // namespace opnpool
} // namespace esphome  