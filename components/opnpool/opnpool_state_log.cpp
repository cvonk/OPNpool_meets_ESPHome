/**
 * @file opnpool_state_log.cpp
 * @brief Pool state: log state as a JSON-formatted string
 *
 * @details
 * This file provides functions to serialize the OPNpool controller's internal state and
 * its subcomponents (system, pump, chlorinator, thermostats, schedules, etc.) into a
 * compact JSON representation for logging.  Each function adds a specific part of the
 * pool state to a cJSON object, using type-safe enum-to-string helpers and value checks
 * to ensure clarity and correctness in the output.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. The code emphasizes maintainability, clear
 * mapping between C++ structures and JSON.
 *
 * Usage: Call state() or the relevant add_* functions to serialize
 * the desired state or subcomponent.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <esp_system.h>
#include <esphome/core/log.h>
#include <cJSON.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "network.h"
#include "opnpool_state.h"

namespace esphome {
namespace opnpool {

namespace opnpoolstatelog {

static cJSON *
_create_item(cJSON * const obj, char const * const key)
{
    if (key == nullptr) {
        return obj;
    }
    cJSON * const item = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, key, item);
    return item;
}

static void
_add_time(cJSON * const obj, char const * const key, poolstate_time_t const * const time)
{
    cJSON_AddStringToObject(obj, key, time_str(time->hour, time->minute));
}

static void
_add_date(cJSON * const obj, char const * const key, poolstate_date_t const * const date)
{
    cJSON_AddStringToObject(obj, key, date_str(date->year, date->month, date->day));
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
    _add_time(item, "time", &tod->time);
    _add_date(item, "date", &tod->date);
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
 * @brief Add system information (time, date, firmware) to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the system object.
 * @param state Pointer to the poolstate_t structure to log.
 */
static void
_dispatch_add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);
    add_time_and_date(item, "tod", &state->system.tod);
    add_version(item, "firmware", &state->system.version);
}

/**
 * @brief Add system information (time, date, firmware) to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the system object.
 * @param state Pointer to the poolstate_t structure to log.
 */
void
add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    _dispatch_add_system(obj, key, state);
}

static void
_add_thermostat(cJSON * const obj, char const * const key,
    poolstate_thermo_t const * const thermostat, 
    bool const showTemp, bool showSp, bool const showSrc, bool const showHeating)
{
    cJSON * const item = _create_item(obj, key);
    if (showTemp) {
        cJSON_AddNumberToObject(item, "temp", thermostat->temp);
    }
    if (showSp) {
        cJSON_AddNumberToObject(item, "sp", thermostat->set_point);
    }
    if (showSrc) {
        network_heat_src_t src = static_cast<network_heat_src_t>(thermostat->heat_src);
        cJSON_AddStringToObject(item, "src", enum_str(src));
    }
    if (showHeating) {
        cJSON_AddBoolToObject(item, "heating", thermostat->heating);
    }
}

/**
 * @brief Add thermostat information to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the thermostat array.
 * @param state Pointer to the poolstate_t structure containing the thermostats.
 */
static void
_dispatch_add_thermos(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    add_thermos(obj, key, state->thermos, true, true, true, true);
}

/**
 * @brief              Add thermostat information to a JSON object for logging.
 *
 * @param obj          The parent JSON object.
 * @param key          The key under which to add the thermostat array.
 * @param thermos      Pointer to the array of poolstate_thermo_t structures.
 * @param showTemp     Whether to include temperature values.
 * @param showSp       Whether to include set point values.
 * @param showSrc      Whether to include heat source values.
 * @param showHeating  Whether to include heating status.
 */
void
add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos,
    bool const showTemp, bool showSp, bool const showSrc, bool const showHeating)
{
    cJSON * const item = _create_item(obj, key);
    for (uint_least8_t ii = 0; ii < enum_count<poolstate_thermo_typ_t>(); ii++, thermos++) {
        _add_thermostat(item, enum_str(static_cast<poolstate_thermo_typ_t>(ii)), thermos,
                        showTemp, showSp, showSrc, showHeating);
    }
}


static void
_add_schedule(cJSON * const obj, char const * const key, poolstate_sched_t const * const sched)
{
    if (sched->active) {
        cJSON * const item = _create_item(obj, key);
        cJSON_AddStringToObject(item, "start", time_str(sched->start / 60, sched->start % 60));
        cJSON_AddStringToObject(item, "stop", time_str(sched->stop / 60, sched->stop % 60));
    }
}

/**
 * @brief       Add schedule information to a JSON object for logging.
 * 
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the schedule array.
 * @param state Pointer to the poolstate_t structure containing the schedules.
 */
static void
_dispatch_add_scheds(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    add_sched(obj, key, state->scheds, true);
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
add_sched(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds,
    bool const showSched)
{
    if (showSched) {
        cJSON * const item = _create_item(obj, key);
        for (uint_least8_t ii = 0; ii < NETWORK_POOL_CIRCUIT_COUNT; ii++, scheds++) {
            if (scheds->active) {
                _add_schedule(item, enum_str(static_cast<network_pool_circuit_t>(ii)), scheds);
            }
        }
    }
}

static void
_add_temp(cJSON * const obj, char const * const key, poolstate_temp_t const * const temp)
{
    if (temp->temp != 0xFF && temp->temp != 0x00) {
        cJSON_AddNumberToObject(obj, key, temp->temp);
    }
}

/**
 * @brief       Add temperature information to a JSON object for logging.
 * 
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the temperature array.
 * @param state Pointer to the poolstate_t structure containing the temperatures.
 */
static void
_dispatch_add_temps(cJSON * const obj, char const * const key, poolstate_t const * state)
{
    cJSON * const item = _create_item(obj, key);
    poolstate_temp_t const * temp = state->temps;
    for (uint_least8_t ii = 0; ii < enum_count<poolstate_temp_typ_t>(); ii++, temp++) {
        _add_temp(item, enum_str(static_cast<poolstate_temp_typ_t>(ii)), temp);
    }
}

/**
 * @brief       Add mode information to a JSON object for logging.
 * 
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the mode array.
 * @param state Pointer to the poolstate_t structure containing the modes.
 */
static void
_dispatch_add_modes(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);

    bool const * is_set = &state->modes.is_set[0];
    for (uint_least8_t ii = 0; ii < NETWORK_POOL_MODE_BITS_COUNT; ii++, is_set++) {
        cJSON_AddBoolToObject(item, enum_str(static_cast<network_pool_mode_bits_t>(ii)), *is_set);
    }
}

static void
_add_circuit_detail(cJSON * const obj, char const * const key, bool const * active)
{
    cJSON * const item = _create_item(obj, key);
    for (uint_least8_t ii = 0; ii < NETWORK_POOL_CIRCUIT_COUNT; ii++, active++) {
        cJSON_AddBoolToObject(item, enum_str(static_cast<network_pool_circuit_t>(ii)), *active);
    }
}

/**
 * @brief       Add circuit information to a JSON object for logging.
 * 
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the circuit array.
 * @param state Pointer to the poolstate_t structure containing the circuits.
 */
static void
_dispatch_add_circuits(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_circuits_t const * const circuits = &state->circuits;
    cJSON * const item = _create_item(obj, key);
    _add_circuit_detail(item, "active", circuits->active);
    _add_circuit_detail(item, "delay", circuits->delay);
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
    _dispatch_add_system(item, "system", state);
    _dispatch_add_temps(item, "temps", state);
    add_thermos(item, "thermos", state->thermos, true, false, true, true);
    add_sched(item, "scheds", state->scheds, true);
    _dispatch_add_modes(item, "modes", state);
    _dispatch_add_circuits(item, "circuits", state);
}

/**
 * @brief       Add pump program value to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pump program value.
 * @param value The pump program value to log.
 */
void
add_pump_program(cJSON * const obj, char const * const key, uint16_t const value)
{
    cJSON_AddNumberToObject(obj, key, value);
}

/**
 * @brief      Add pump control mode to a JSON object for logging.
 *
 * @param obj  The parent JSON object.
 * @param key  The key under which to add the pump control value.
 * @param ctrl The pump control value to log (0x00 = local, 0xFF = remote, other = numeric).
 */
void
add_pump_ctrl(cJSON * const obj, char const * const key, uint8_t const ctrl)
{
    char const * str;
    switch (ctrl) {
        case 0x00: 
            str = "local"; 
            break;
        case 0xFF:                                             
            str = "remote"; 
            break;
        default: {
            str = uint8_str(ctrl);
            break;
        }
    }
    cJSON_AddStringToObject(obj, key, str);
}

/**
 * @brief      Add pump mode to a JSON object for logging.
 *
 * @param obj  The parent JSON object.
 * @param key  The key under which to add the pump mode value.
 * @param mode The pump mode value to log (as enum).
 */
void
add_pump_mode(
    cJSON * const obj, char const * const key, network_pump_mode_t const mode)
{
    cJSON_AddStringToObject(obj, key, enum_str(mode));
}

static void
_addPumpStateToObject(cJSON * const obj, char const * const key, network_pump_state_t const state)
{
    cJSON_AddStringToObject(obj, key, enum_str(state));
}

/**
 * @brief         Add pump running status to a JSON object for logging.
 *
 * @param obj     The parent JSON object.
 * @param key     The key under which to add the running status.
 * @param running The pump running status to log (true if running).
 */
void
add_pump_running(cJSON * const obj, char const * const key, bool const running)
{
    cJSON_AddBoolToObject(obj, key, running);
}

/**
 * @brief       Add pump information to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pump object.
 * @param state Pointer to the poolstate_t structure containing the pump information.
 */
static void
_dispatch_add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_pump_t const * const pump = &state->pump;
    cJSON * const item = _create_item(obj, key);
    _add_time(item, "time", &pump->time);
    add_pump_mode(item, "mode", pump->mode);
    add_pump_running(item, "running", pump->running);
    _addPumpStateToObject(item, "state", pump->state);
    cJSON_AddNumberToObject(item, "power", pump->power);
    cJSON_AddNumberToObject(item, "speed", pump->speed);
    if (pump->flow) {
        cJSON_AddNumberToObject(item, "flow", pump->flow);
    }
    if (pump->level) {
        cJSON_AddNumberToObject(item, "level", pump->level);
    }
    cJSON_AddNumberToObject(item, "error", pump->error);
    cJSON_AddNumberToObject(item, "timer", pump->timer);
}

/**
 * @brief       Add pump information to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the pump object.
 * @param state Pointer to the poolstate_t structure containing the pump information.
 */
void
add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    _dispatch_add_pump(obj, key, state);
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
    cJSON_AddNumberToObject(item, "salt", chlor->salt);
    cJSON_AddStringToObject(item, "status", enum_str(chlor->status));
}

/**
 * @brief       Add chlorinator information to a JSON object for logging.
 *
 * @param obj   The parent JSON object.
 * @param key   The key under which to add the chlorinator object.
 * @param state Pointer to the poolstate_t structure containing the chlorinator information.
 */
static void
_dispatch_add_chlor(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_chlor_t const * const chlor = &state->chlor;
    cJSON * const item = _create_item(obj, key);
    cJSON_AddStringToObject(item, "name", chlor->name);
    cJSON_AddNumberToObject(item, "level", chlor->level);
    cJSON_AddNumberToObject(item, "salt", chlor->salt);
    cJSON_AddStringToObject(item, "status", enum_str(chlor->status));
}

/**
 * and finally .. poolstate itself
 **/

using poolstate_json_fnc_t = void (*)(
    cJSON * const obj, char const * const key,
    poolstate_t const * const state
);

struct poolstate_json_dispatch_t {
    poolstate_elem_typ_t const  typ;
    char const * const      name;
    poolstate_json_fnc_t const  fnc;
};

static poolstate_json_dispatch_t _dispatches[] = {
    { poolstate_elem_typ_t::SYSTEM,   "system",   _dispatch_add_system   },
    { poolstate_elem_typ_t::TEMP,     "temps",    _dispatch_add_temps    },
    { poolstate_elem_typ_t::THERMO,   "thermos",  _dispatch_add_thermos  },
    { poolstate_elem_typ_t::PUMP,     "pump",     _dispatch_add_pump     },
    { poolstate_elem_typ_t::CHLOR,    "chlor",    _dispatch_add_chlor    },
    { poolstate_elem_typ_t::CIRCUITS, "circuits", _dispatch_add_circuits },
    { poolstate_elem_typ_t::SCHED,    "scheds",   _dispatch_add_scheds   },
    { poolstate_elem_typ_t::MODES,    "modes",    _dispatch_add_modes    },
};

/**
 * @brief        Generate a JSON string representing the pool state or a specific element.
 *
 * @param state  Pointer to the pool state structure.
 * @param typ    The poolstate element type to log, or ALL for the full state.
 * @return       Unformatted JSON string (caller must free).
 */
char const *
state(poolstate_t const * const state, poolstate_elem_typ_t const typ)
{
    name_reset_idx();

    cJSON * const obj = cJSON_CreateObject();

    poolstate_json_dispatch_t const * dispatch = _dispatches;
    for (uint_least8_t ii = 0; ii < ARRAY_SIZE(_dispatches); ii++, dispatch++) {
        bool const all_types = typ == poolstate_elem_typ_t::ALL;
        if (typ == dispatch->typ || all_types) {

            dispatch->fnc(obj, all_types ? dispatch->name : NULL, state);
        }
    }
    char const * const json = cJSON_PrintUnformatted(obj);
	cJSON_Delete(obj);
	return json;  // caller MUST free
}   

}  // namespace opnpoolstate_log

}  // namespace opnpool
}  // namespace esphome