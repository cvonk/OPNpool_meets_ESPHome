/**
 * @file poolstate_rx_log.cpp
 * @brief PoolState: log state as a JSON-formatted string
 *
 * @details
 * This file provides functions to serialize the OPNpool controller's internal state and
 * its subcomponents (system, pump, chlorinator, thermostats, schedules, etc.) into a
 * compact JSON representation for logging.  Each function adds a specific part of the
 * pool state to a cJSON object, using type-safe enum-to-string helpers and value checks
 * to ensure clarity and correctness in the output.
 *
 * These functions are kept seperate from poolstate_rx.cpp because their purpose is
 * to provide logging functionality, and separating them helps to avoid making that file
 * unwieldy large.
 * 
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. The code emphasizes maintainability, clear
 * mapping between C++ structures and JSON.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/log.h>
#include <cJSON.h>
#include <string.h>
#include <cstddef>

#include "to_str.h"
#include "enum_helpers.h"
#include "network.h"
#include "poolstate.h"
#include "poolstate_rx_log.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

namespace poolstate_rx {
namespace poolstate_rx_log {

[[nodiscard]] static cJSON *
_create_item(cJSON * const obj, char const * const key)
{
    if (key == nullptr) {
        return obj;
    }
    cJSON * const item = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, key, item);
    return item;
}

    // add system information (time, date, firmware) to a JSON object
void
_add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);
 
    add_time_and_date(item, KEY_TOD, &state->system.tod);
    add_version(item, KEY_FIRMWARE, &state->system.version);
}

    // add active circuit information to a JSON object
static void
_add_circuit_active(cJSON * const obj, char const * const key, poolstate_circuit_t const * circuit)
{
    cJSON * const item = _create_item(obj, key);

    for (auto typ : magic_enum::enum_values<network_pool_circuit_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), circuit->active.value);
        circuit++;
    }
}

    // add delay circuit information to a JSON object
static void
_add_circuit_delay(cJSON * const obj, char const * const key, poolstate_circuit_t const * circuit)
{
    cJSON * const item = _create_item(obj, key);

    for (auto typ : magic_enum::enum_values<network_pool_circuit_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), circuit->delay.value);
        circuit++;
    }
}

    // add circuit information to a JSON object
static void
_add_circuits(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);

    _add_circuit_active(item, KEY_ACTIVE, state->circuits);
    _add_circuit_delay(item, KEY_DELAY, state->circuits);
}

    // add mode information to a JSON object
static void
_add_modes(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);

    poolstate_bool_t const * mode = state->modes;
    for (auto typ : magic_enum::enum_values<network_pool_mode_bits_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), mode->value);
        mode++;
    }
}

    // add temperatures information to a JSON object
static void
_add_temps(cJSON * const obj, char const * const key, poolstate_t const * state)
{
    cJSON * const item = _create_item(obj, key);
    
    poolstate_uint8_t const * temp = state->temps;    
    for (auto typ : magic_enum::enum_values<poolstate_temp_typ_t>()) {
        if (temp->value != 0xFF && temp->value != 0x00) {
            cJSON_AddNumberToObject(item, enum_str(typ), temp->value);
        }
        temp++;
    }
}

static void
_add_pump_mode(cJSON * const obj, char const * const key, network_pump_mode_t const mode)
{
    cJSON_AddStringToObject(obj, key, enum_str(mode));
}

static void
_add_pump_running(cJSON * const obj, char const * const key, bool const running)
{
    cJSON_AddBoolToObject(obj, key, running);
}


/**
 * @brief Add time and date information to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the time and date object.
 * @param tod Pointer to the poolstate_tod_t structure containing the time and date.
 */
void
add_time_and_date(cJSON * const obj, char const * const key, poolstate_tod_t const * const tod)
{
    cJSON * const item = _create_item(obj, key);

    cJSON_AddStringToObject(item, KEY_TIME, time_str(tod->time.hour, tod->time.minute));    
    cJSON_AddStringToObject(item, KEY_DATE, date_str(tod->date.year, tod->date.month, tod->date.day));
}

/**
 * @brief Add version information to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the version string.
 * @param version Pointer to the poolstate_version_t structure containing the version information.
 */
void
add_version(cJSON * const obj, char const * const key, poolstate_version_t const * const version)
{
    cJSON_AddStringToObject(obj, key, version_str(version->major, version->minor));
}

/**
 * @brief              Add thermostat information to a JSON object for logging.
 *
 * @param obj          The parent JSON object.
 * @param key          The key under which to add the thermostat array.
 * @param thermos      Pointer to the array of poolstate_thermo_t structures.
 * @param showTemp     Whether to include temperature values.
 * @param showSp       Whether to include set point values.
 * @param showHeating  Whether to include heating status.
 */
void
add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos,
            bool const showTemp, bool showSp, bool const showHeating)
{
    cJSON * const item = _create_item(obj, key);

    for (auto typ : magic_enum::enum_values<poolstate_thermo_typ_t>()) {

        cJSON * const sub_item = _create_item(item, enum_str(typ));

        if (showTemp) {
            cJSON_AddNumberToObject(sub_item, KEY_TEMP, thermos->temp_in_f.value);
        }

        if (showSp) {
            cJSON_AddNumberToObject(sub_item, KEY_SP, thermos->set_point_in_f.value);
        }
        cJSON_AddStringToObject(sub_item, KEY_SRC, enum_str(thermos->heat_src.value));

        if (showHeating) {
            cJSON_AddBoolToObject(sub_item, KEY_HEATING, thermos->heating.value);
        }
        thermos++;
    }
}

/**
 * @brief           Add schedule information to a JSON object for logging.
 *
 * @param obj       The parent JSON object.
 * @param key       The key under which to add the schedule array.
 * @param scheds    Pointer to the array of poolstate_sched_t structures.
 * @param showSched Whether to include schedule information.
 */
void
add_scheds(cJSON * const obj, char const * const key, poolstate_sched_t const * sched)
{
    cJSON * const item = _create_item(obj, key);

    for (auto circuit : magic_enum::enum_values<network_pool_circuit_t>()) {
        if (sched->active) {
            if (sched->active) {
                cJSON * const sub_item = _create_item(item, enum_str(circuit));

                cJSON_AddStringToObject(sub_item, KEY_START, time_str(sched->start / 60, sched->start % 60));
                cJSON_AddStringToObject(sub_item, KEY_STOP, time_str(sched->stop / 60, sched->stop % 60));
            }
        }
        sched++;
    }
}

/**
 * @brief       Add the controller pool state to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pool state object.
 * @param state Pointer to the poolstate_t structure to log.
 */
void
add_state(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);

    add_thermos(item, KEY_THERMOS, state->thermos, true, false, true);
    add_scheds(item, KEY_SCHEDS, state->scheds);
    _add_system(item, KEY_SYSTEM, state);
    _add_temps(item, KEY_TEMPS, state);
    _add_modes(item, KEY_MODES, state);
    _add_circuits(item, KEY_CIRCUITS, state);
}

/**
 * @brief       Add pump information to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pump object.
 * @param state Pointer to the poolstate_t structure containing the pump information.
 */
void
add_pump(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, poolstate_pump_t const * const pump)
{
    cJSON * const item1 = _create_item(obj, key);
    cJSON * const item = _create_item(item1, key);

    _add_pump_mode(item, KEY_MODE, pump->mode.value);
    _add_pump_running(item, KEY_RUNNING, pump->running.value);

    cJSON_AddStringToObject(item, KEY_TIME, time_str(pump->time.hour, pump->time.minute));    
    cJSON_AddStringToObject(item, KEY_STATE, enum_str(pump->state.value));
    cJSON_AddNumberToObject(item, KEY_POWER, pump->power.value);
    cJSON_AddNumberToObject(item, KEY_SPEED, pump->speed.value);
    if (pump->flow.value) {
        cJSON_AddNumberToObject(item, KEY_FLOW, pump->flow.value);
    }
    if (pump->level.value) {
        cJSON_AddNumberToObject(item, KEY_LEVEL, pump->level.value);
    }
    cJSON_AddNumberToObject(item, KEY_ERROR, pump->error.value);
    cJSON_AddNumberToObject(item, KEY_TIMER, pump->timer.value);
}

/**
 * @brief       Add pump program value to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pump program value.
 * @param value The pump program value to log.
 */
void
add_pump_program(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, uint16_t const value)
{
    cJSON * const item = _create_item(obj, enum_str(dev_id));

    cJSON_AddNumberToObject(item, key, value);
}

/**
 * @brief      Add pump control mode to a JSON object for logging.
 *
 * @param obj  The parent JSON object.
 * @param key  The key under which to add the pump control value.
 * @param ctrl The pump control value to log (0x00 = local, 0xFF = remote, other = numeric).
 */
void
add_pump_ctrl(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, network_pump_ctrl_t const ctrl)
{
    cJSON * const item = _create_item(obj, enum_str(dev_id));

    cJSON_AddStringToObject(item, key, enum_str(ctrl));
}

/**
 * @brief      Add pump mode to a JSON object for logging.
 *
 * @param obj  The parent JSON object.
 * @param key  The key under which to add the pump mode value.
 * @param mode The pump mode value to log (as enum).
 */
void
add_pump_mode(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, network_pump_mode_t const mode)
{
    cJSON * const item = _create_item(obj, enum_str(dev_id));

    _add_pump_mode(item, key, mode);
}

/**
 * @brief         Add pump running status to a JSON object for logging.
 *
 * @param obj     The parent JSON object.
 * @param key     The key under which to add the running status.
 * @param running The pump running status to log (true if running).
 */
void
add_pump_running(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, bool const running)
{
    cJSON * const item = _create_item(obj, enum_str(dev_id));

    _add_pump_running(item, key, running);
}

/**
 * @brief       Add chlorinator response information to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the chlorinator response object.
 * @param chlor Pointer to the poolstate_chlor_t structure to log.
 */
void
add_chlor_resp(cJSON * const obj, char const * const key, poolstate_chlor_t const * const chlor)
{
    cJSON * const item = _create_item(obj, key);

    cJSON_AddNumberToObject(item, KEY_SALT, chlor->salt.value);
    cJSON_AddStringToObject(item, KEY_STATUS, enum_str(chlor->status.value));
}


}  // namespace poolstate_rx_log
}  // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome