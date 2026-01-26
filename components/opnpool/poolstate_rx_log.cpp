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
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

namespace poolstate_rx {
namespace poolstate_rx_log {

    // JSON key names
constexpr char const * const KEY_TIME     = "time";
constexpr char const * const KEY_DATE     = "date";
constexpr char const * const KEY_FIRMWARE = "firmware";
constexpr char const * const KEY_TOD      = "tod";
constexpr char const * const KEY_TEMP     = "temp";
constexpr char const * const KEY_SP       = "sp";
constexpr char const * const KEY_SRC      = "src";
constexpr char const * const KEY_HEATING  = "heating";
constexpr char const * const KEY_START    = "start";
constexpr char const * const KEY_STOP     = "stop";
constexpr char const * const KEY_ACTIVE   = "active";
constexpr char const * const KEY_DELAY    = "delay";
constexpr char const * const KEY_SYSTEM   = "system";
constexpr char const * const KEY_TEMPS    = "temps";
constexpr char const * const KEY_THERMOS  = "thermos";
constexpr char const * const KEY_PUMP     = "pump";
constexpr char const * const KEY_CHLOR    = "chlor";
constexpr char const * const KEY_CIRCUITS = "circuits";
constexpr char const * const KEY_SCHEDS   = "scheds";
constexpr char const * const KEY_MODES    = "modes";
constexpr char const * const KEY_NAME     = "name";
constexpr char const * const KEY_LEVEL    = "level";
constexpr char const * const KEY_SALT     = "salt";
constexpr char const * const KEY_STATUS   = "status";
constexpr char const * const KEY_MODE     = "mode";
constexpr char const * const KEY_RUNNING  = "running";
constexpr char const * const KEY_STATE    = "state";
constexpr char const * const KEY_POWER    = "power";
constexpr char const * const KEY_SPEED    = "speed";
constexpr char const * const KEY_FLOW     = "flow";
constexpr char const * const KEY_ERROR    = "error";
constexpr char const * const KEY_TIMER    = "timer";

inline cJSON *
_create_item(cJSON * const obj, char const * const key)
{
    if (key == nullptr) {
        return obj;
    }
    cJSON * const item = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, key, item);
    return item;
}

    // add system information (time, date, firmware) to a JSON object for logging.
static void
_add_system_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);

    add_time_and_date(item, KEY_TOD, &state->system.tod);
    add_version(item, KEY_FIRMWARE, &state->system.version);
}

static void
_add_thermostat(cJSON * const obj, char const * const key, poolstate_thermo_t const * const thermostat, 
    bool const showTemp, bool showSp, bool const showHeating)
{
    cJSON * const item = _create_item(obj, key);
    if (showTemp) {
        cJSON_AddNumberToObject(item, KEY_TEMP, thermostat->temp_in_f.value);
    }
    if (showSp) {
        cJSON_AddNumberToObject(item, KEY_SP, thermostat->set_point_in_f.value);
    }
    network_heat_src_t src = thermostat->heat_src.value;
    cJSON_AddStringToObject(item, KEY_SRC, enum_str(src));

    if (showHeating) {
        cJSON_AddBoolToObject(item, KEY_HEATING, thermostat->heating.value);
    }
}

    // add thermostat information to a JSON object for logging.
static void
_add_thermos_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    add_thermos(obj, key, state->thermos, true, true, true);
}

static void
_add_schedule(cJSON * const obj, char const * const key, poolstate_sched_t const * const sched)
{
    if (sched->active) {
        cJSON * const item = _create_item(obj, key);
        cJSON_AddStringToObject(item, KEY_START, time_str(sched->start / 60, sched->start % 60));
        cJSON_AddStringToObject(item, KEY_STOP, time_str(sched->stop / 60, sched->stop % 60));
    }
}

    // add schedule information to a JSON object for logging.
static void
_add_scheds_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    add_sched(obj, key, state->scheds, true);
}

static void
_add_temp(cJSON * const obj, char const * const key, poolstate_uint8_t const * const temp)
{
    if (temp->value != 0xFF && temp->value != 0x00) {
        cJSON_AddNumberToObject(obj, key, temp->value);
    }
}

    // add temperature information to a JSON object for logging.
static void
_add_temps_dispatch(cJSON * const obj, char const * const key, poolstate_t const * state)
{
    cJSON * const item = _create_item(obj, key);
    poolstate_uint8_t const * temp = state->temps;
    
    for (auto typ : magic_enum::enum_values<poolstate_temp_typ_t>()) {
        _add_temp(item, enum_str(typ), temp);
        ++temp;
    }
}

    // add mode information to a JSON object for logging.
static void
_add_modes_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);
    poolstate_bool_t const * mode = state->modes;

    for (auto typ : magic_enum::enum_values<network_pool_mode_bits_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), mode->value);
        ++mode;
    }
}

static void
_add_circuit_active_detail(cJSON * const obj, char const * const key, poolstate_circuit_t const * circuit)
{
    cJSON * const item = _create_item(obj, key);

    for (auto typ : magic_enum::enum_values<network_pool_circuit_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), circuit->active.value);
        circuit++;
    }
}

static void
_add_circuit_delay_detail(cJSON * const obj, char const * const key, poolstate_circuit_t const * circuit)
{
    cJSON * const item = _create_item(obj, key);
    (void)item;
    for (auto typ : magic_enum::enum_values<network_pool_circuit_t>()) {
        cJSON_AddBoolToObject(item, enum_str(typ), circuit->delay.value);
        circuit++;
    }
}

    // add circuit information to a JSON object for logging.
static void
_add_circuits_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_circuit_t const * const circuits = state->circuits;
    cJSON * const item = _create_item(obj, key);
    _add_circuit_active_detail(item, "active", circuits);
    _add_circuit_delay_detail(item, "delay", circuits);
}

static void
_addPumpStateToObject(cJSON * const obj, char const * const key, network_pump_state_t const state)
{
    cJSON_AddStringToObject(obj, key, enum_str(state));
}

    // add pump information to a JSON object for logging.
static void
_add_pump_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_pump_t const * const pump = &state->pump;
    cJSON * const item = _create_item(obj, key);

    cJSON_AddStringToObject(item, KEY_TIME, time_str(pump->time.hour, pump->time.minute));    
    add_pump_mode(item, KEY_MODE, pump->mode.value);
    add_pump_running(item, KEY_RUNNING, pump->running.value);
    _addPumpStateToObject(item, KEY_STATE, pump->state.value);
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

    // add chlorinator information to a JSON object for logging.
static void
_add_chlor_dispatch(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_chlor_t const * const chlor = &state->chlor;
    cJSON * const item = _create_item(obj, key);
    cJSON_AddStringToObject(item, KEY_NAME, chlor->name.value);
    cJSON_AddNumberToObject(item, KEY_LEVEL, chlor->level.value);
    cJSON_AddNumberToObject(item, KEY_SALT, chlor->salt.value);
    cJSON_AddStringToObject(item, KEY_STATUS, enum_str(chlor->status.value));
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
 * @brief Add system information (time, date, firmware) to a JSON object for logging.
 * 
 * @param obj The parent JSON object.
 * @param key The key under which to add the system object.
 * @param state Pointer to the poolstate_t structure to log.
 */
void
add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    _add_system_dispatch(obj, key, state);
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
        _add_thermostat(item, enum_str(typ), thermos, showTemp, showSp, showHeating);
        ++thermos;
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
add_sched(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds,
    bool const showSched)
{
    if (showSched) {
        cJSON * const item = _create_item(obj, key);
        for (auto circuit : magic_enum::enum_values<network_pool_circuit_t>()) {
            if (scheds->active) {
                _add_schedule(item, enum_str(circuit), scheds);
            }
            ++scheds;
        }
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
    _add_system_dispatch(item, "system", state);
    _add_temps_dispatch(item, "temps", state);
    add_thermos(item, "thermos", state->thermos, true, false, true);
    add_sched(item, "scheds", state->scheds, true);
    _add_modes_dispatch(item, "modes", state);
    _add_circuits_dispatch(item, "circuits", state);
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
    _add_pump_dispatch(obj, key, state);
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
add_pump_ctrl(cJSON * const obj, char const * const key, network_pump_ctrl_t const ctrl)
{
    cJSON_AddStringToObject(obj, key, enum_str(ctrl));
}

/**
 * @brief      Add pump mode to a JSON object for logging.
 *
 * @param obj  The parent JSON object.
 * @param key  The key under which to add the pump mode value.
 * @param mode The pump mode value to log (as enum).
 */
void
add_pump_mode(cJSON * const obj, char const * const key, network_pump_mode_t const mode)
{
    cJSON_AddStringToObject(obj, key, enum_str(mode));
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
    { poolstate_elem_typ_t::SYSTEM,   "system",   _add_system_dispatch   },
    { poolstate_elem_typ_t::TEMP,     "temps",    _add_temps_dispatch    },
    { poolstate_elem_typ_t::THERMO,   "thermos",  _add_thermos_dispatch  },
    { poolstate_elem_typ_t::PUMP,     "pump",     _add_pump_dispatch     },
    { poolstate_elem_typ_t::CHLOR,    "chlor",    _add_chlor_dispatch    },
    { poolstate_elem_typ_t::CIRCUITS, "circuits", _add_circuits_dispatch },
    { poolstate_elem_typ_t::SCHED,    "scheds",   _add_scheds_dispatch   },
    { poolstate_elem_typ_t::MODES,    "modes",    _add_modes_dispatch    },
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

    for (const auto dispatch : _dispatches) {
        bool const all_types = typ == poolstate_elem_typ_t::ALL;
        if (typ == dispatch.typ || all_types) {
            dispatch.fnc(obj, all_types ? dispatch.name : NULL, state);
        }
    }
    
    char const * const json = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    return json;  // caller MUST free
}   

}  // namespace poolstate_rx_log
}  // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome