/**
 * @brief OPNpool - Pool state: log state as a JSON-formatted string
 *
 * © Copyright 2014, 2019, 2022, 2026, Coert Vonk
 * 
 * This file is part of OPNpool.
 * OPNpool is free software: you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 * OPNpool is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with OPNpool. 
 * If not, see <https://www.gnu.org/licenses/>.
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: Copyright 2014,2019,2022,2026 Coert Vonk
 */

#include <string.h>
#include <esp_system.h>
#include <esphome/core/log.h>
#include <cJSON.h>

#include "utils.h"
#include "network.h"
#include "opnpoolstate.h"

namespace esphome {
namespace opnpool {

static cJSON *
_create_item(cJSON * const obj, char const * const key)
{
    if (key == NULL) {
        return obj;
    }
    cJSON * const item = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, key, item);
    return item;
}

/**
 * poolstate->system
 **/

static void
_add_time(cJSON * const obj, char const * const key, poolstate_time_t const * const time)
{
    cJSON_AddStringToObject(obj, key, network_ctrl_time_str(time->hour, time->minute));
}

static void
_add_date(cJSON * const obj, char const * const key, poolstate_date_t const * const date)
{
    cJSON_AddStringToObject(obj, key, network_ctrl_date_str(date->year, date->month, date->day));
}

void
opnpoolstate_log_add_tod(cJSON * const obj, char const * const key, poolstate_tod_t const * const tod)
{
    cJSON * const item = _create_item(obj, key);
    _add_time(item, "time", &tod->time);
    _add_date(item, "date", &tod->date);
}

void
opnpoolstate_log_add_version(cJSON * const obj, char const * const key, poolstate_version_t const * const version)
{
    cJSON_AddStringToObject(obj, key, network_ctrl_version_str(version->major, version->minor));
}

static void
_add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);
    opnpoolstate_log_add_tod(item, "tod", &state->system.tod);
    opnpoolstate_log_add_version(item, "firmware", &state->system.version);
}

void
opnpoolstate_log_add_system(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    _add_system(obj, key, state);
}

/**
 * poolstate->thermos
 **/

static void
_add_thermostat(cJSON * const obj, char const * const key, poolstate_thermo_t const * const thermostat,
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
        cJSON_AddStringToObject(item, "src", network_heat_src_str(static_cast<network_heat_src_t>(thermostat->heat_src)));
    }
    if (showHeating) {
        cJSON_AddBoolToObject(item, "heating", thermostat->heating);
    }
}

static void
_add_thermos(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    opnpoolstate_log_add_thermos(obj, key, state->thermos, true, true, true, true);
}

void
opnpoolstate_log_add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos,
                             bool const showTemp, bool showSp, bool const showSrc, bool const showHeating)
{
    cJSON * const item = _create_item(obj, key);
    for (uint8_t ii = 0; ii < POOLSTATE_THERMO_TYP_COUNT; ii++, thermos++) {
        _add_thermostat(item, poolstate_str_thermo_str(static_cast<poolstate_thermo_typ_t>(ii)), thermos,
                        showTemp, showSp, showSrc, showHeating);
    }
}


/**
 * poolstate->scheds
 **/

static void
_add_schedule(cJSON * const obj, char const * const key, poolstate_sched_t const * const sched)
{
    if (sched->active) {
        cJSON * const item = _create_item(obj, key);
        cJSON_AddStringToObject(item, "start", network_ctrl_time_str(sched->start / 60, sched->start % 60));
        cJSON_AddStringToObject(item, "stop", network_ctrl_time_str(sched->stop / 60, sched->stop % 60));
    }
}

static void
_add_scheds(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    opnpoolstate_log_add_sched(obj, key, state->scheds, true);
}

void
opnpoolstate_log_add_sched(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds, bool const showSched)
{
    if (showSched) {
        cJSON * const item = _create_item(obj, key);
        for (uint8_t ii = 0; ii < NETWORK_POOL_MODE_COUNT; ii++, scheds++) {
            _add_schedule(item, network_pool_circuit_str(static_cast<network_pool_circuit_t>(ii)), scheds);
        }
    }
}

/**
 * poolstate->temps
 **/

static void
_add_temp(cJSON * const obj, char const * const key, poolstate_temp_t const * const temp)
{
    if (temp->temp != 0xFF && temp->temp != 0x00) {
        cJSON_AddNumberToObject(obj, key, temp->temp);
    }
}

static void
_add_temps(cJSON * const obj, char const * const key, poolstate_t const * state)
{
    cJSON * const item = _create_item(obj, key);
    poolstate_temp_t const * temp = state->temps;
    for (uint8_t ii = 0; ii < POOLSTATE_TEMP_TYP_COUNT; ii++, temp++) {
        _add_temp(item, poolstate_str_temp_str(static_cast<poolstate_temp_typ_t>(ii)), temp);
    }
}

/**
 * poolstate->modes
 **/

static void
_add_modes(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_modes_t const * const modes = &state->modes;
    cJSON * const item = _create_item(obj, key);

    bool const * set = modes->set;
    for (uint8_t ii = 0; ii < NETWORK_POOL_MODE_COUNT; ii++, set++) {
        cJSON_AddBoolToObject(item, network_pool_mode_str(static_cast<network_pool_mode_t>(ii)), *set);
    }
}

/**
 * poolstate->circuits, poolstate->delay 
 **/

static void
_add_circuit_detail(cJSON * const obj, char const * const key, bool const * active)
{
    cJSON * const item = _create_item(obj, key);
    for (uint8_t ii = 0; ii < NETWORK_POOL_MODE_COUNT; ii++, active++) {
        cJSON_AddBoolToObject(item, network_pool_circuit_str(static_cast<network_pool_circuit_t>(ii)), *active);
    }
}

void
_add_circuits(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_circuits_t const * const circuits = &state->circuits;
    cJSON * const item = _create_item(obj, key);
    _add_circuit_detail(item, "active", circuits->active);
    _add_circuit_detail(item, "delay", circuits->delay);
}

/**
 * poolstate
 **/

void
opnpoolstate_log_add_state(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    cJSON * const item = _create_item(obj, key);
    _add_system(item, "system", state);
    _add_temps(item, "temps", state);
    opnpoolstate_log_add_thermos(item, "thermos", state->thermos, true, false, true, true);
    opnpoolstate_log_add_sched(item, "scheds", state->scheds, true);
    _add_modes(item, "modes", state);
    _add_circuits(item, "circuits", state);
}

/**
 * poolstate->pump
 **/

void
opnpoolstate_log_add_pump_program(cJSON * const obj, char const * const key, uint16_t const value)
{
    cJSON_AddNumberToObject(obj, key, value);
}

void
opnpoolstate_log_add_pump_ctrl(cJSON * const obj, char const * const key, uint8_t const ctrl)
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
            thread_local char hex_buffer[3];  // "XX\0"
            snprintf(hex_buffer, sizeof(hex_buffer), "%02x", ctrl);
            str = hex_buffer;
            break;
        }
    }
    cJSON_AddStringToObject(obj, key, str);
}

void
opnpoolstate_log_add_pump_operation_mode(cJSON * const obj, char const * const key, uint8_t const mode)
{
    cJSON_AddStringToObject(obj, key, network_pump_operation_mode_str(static_cast<network_pump_operation_mode_t>(mode)));
}

static void
_addPumpStateToObject(cJSON * const obj, char const * const key, uint8_t const state)
{
    cJSON_AddStringToObject(obj, key, network_pump_state_str(static_cast<network_pump_state_t>(state)));
}

void
opnpoolstate_log_add_pump_running(cJSON * const obj, char const * const key, bool const running)
{
    cJSON_AddBoolToObject(obj, key, running);
}

#if 0
void
cJSON_AddPumpStatusToObject(cJSON * const obj, char const * const key, poolstate_pump_t const * const pump)
{
    cJSON * const item = _create_item(obj, key);
    opnpoolstate_log_add_pump_running(item, "running", pump->running);
    opnpoolstate_log_add_pump_operation_mode(item, "mode", pump->mode);
    _addPumpStateToObject(item, "state", pump->state);
}
#endif

static void
_add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_pump_t const * const pump = &state->pump;
    cJSON * const item = _create_item(obj, key);
    _add_time(item, "time", &pump->time);
    opnpoolstate_log_add_pump_operation_mode(item, "mode", pump->mode);
    opnpoolstate_log_add_pump_running(item, "running", pump->running);
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

void
opnpoolstate_log_add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    _add_pump(obj, key, state);
}

/**
 * poolstate->chlor
 **/

void
opnpoolstate_log_add_chlor_resp(cJSON * const obj, char const * const key, poolstate_chlor_t const * const chlor)
{
    cJSON * const item = _create_item(obj, key);
    cJSON_AddNumberToObject(item, "salt", chlor->salt);
    cJSON_AddStringToObject(item, "status", poolstate_str_chlor_status_str(chlor->status));
}

static void
_add_chlor(cJSON * const obj, char const * const key, poolstate_t const * const state)
{
    poolstate_chlor_t const * const chlor = &state->chlor;
    cJSON * const item = _create_item(obj, key);
    cJSON_AddStringToObject(item, "name", chlor->name);
    cJSON_AddNumberToObject(item, "level", chlor->level);
    cJSON_AddNumberToObject(item, "salt", chlor->salt);
    cJSON_AddStringToObject(item, "status", poolstate_str_chlor_status_str(chlor->status));
}

/**
 * and finally .. poolstate itself
 **/

typedef void (* poolstate_json_fnc_t)(cJSON * const obj, char const * const key, poolstate_t const * const state);

typedef struct poolstate_json_dispatch_t {
    poolstate_elem_typ_t const  typ;
    char const * const      name;
    poolstate_json_fnc_t const  fnc;
} poolstate_json_dispatch_t;

static poolstate_json_dispatch_t _dispatches[] = {
    { poolstate_elem_typ_t::SYSTEM,   "system",   _add_system   },
    { poolstate_elem_typ_t::TEMP,     "temps",    _add_temps    },
    { poolstate_elem_typ_t::THERMO,   "thermos",  _add_thermos  },
    { poolstate_elem_typ_t::PUMP,     "pump",     _add_pump     },
    { poolstate_elem_typ_t::CHLOR,    "chlor",    _add_chlor    },
    { poolstate_elem_typ_t::CIRCUITS, "circuits", _add_circuits },
    { poolstate_elem_typ_t::SCHED,    "scheds",   _add_scheds   },
    { poolstate_elem_typ_t::MODES,    "modes",    _add_modes    },
};

char const *
opnpoolstate_log_state(poolstate_t const * const state, poolstate_elem_typ_t const typ)
{
    name_reset_idx();
    cJSON * const obj = cJSON_CreateObject();
    poolstate_json_dispatch_t const * dispatch = _dispatches;
    for (uint8_t ii = 0; ii < ARRAY_SIZE(_dispatches); ii++, dispatch++) {
        bool const all_types = typ == poolstate_elem_typ_t::ALL;
        if (typ == dispatch->typ || all_types) {

            dispatch->fnc(obj, all_types ? dispatch->name : NULL, state);
        }
    }
    char const * const json = cJSON_PrintUnformatted(obj);
	cJSON_Delete(obj);
	return json;  // caller MUST free
}

}  // namespace opnpool
}  // namespace esphome