/**
 * @brief OPNpool - Pool state: support, state to cJSON
 *
 * © Copyright 2014, 2019, 2022, 2026 Coert Vonk
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
#include <esp_ota_ops.h>
#include <esp_flash.h>
#include <esphome/core/log.h>
#include <cJSON.h>
#include <assert.h>

#include "utils.h"
#include "network.h"
#include "opnpoolstate.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpoolstate_get";

static void
_alloc_str(char * * const value, char const * const str)
{
    assert( asprintf(value, "%s", str) >= 0);
}

static void
_alloc_strs(char * * const value, char const * const str1, char const * const str2)
{
    assert( asprintf(value, "%s - %s", str1, str2) >= 0);
}

static void
_alloc_uint(char * * const value, uint16_t const num)
{
    assert( asprintf(value, "%u", num) >= 0);
}

static void
_alloc_bool(char * * const value, bool const num)
{
    assert( asprintf(value, "%s", num ? "ON" : "OFF") >= 0);
}

static esp_err_t
_system(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * value)
{
    poolstate_system_t const * const system = &state->system;
    poolstate_elem_system_typ_t const elem_system_typ = static_cast<poolstate_elem_system_typ_t>(typ);

    switch (elem_system_typ) {
        case poolstate_elem_system_typ_t::CTRL_VERSION:
            _alloc_str(value, network_ctrl_version_str(system->version.major, system->version.minor));
            break;
        case poolstate_elem_system_typ_t::IF_VERSION: {
            esp_partition_t const * const running_part = esp_ota_get_running_partition();
            esp_app_desc_t running_app_info;
            ESP_ERROR_CHECK(esp_ota_get_partition_description(running_part, &running_app_info));
            _alloc_str(value, running_app_info.version);
            break;
        }
        case poolstate_elem_system_typ_t::TIME:
            _alloc_str(value, network_ctrl_time_str(system->tod.time.hour, system->tod.time.minute));
            break;
        default:
            ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t
_temp(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * const value)
{
    poolstate_temp_t const * const temp = &state->temps[idx];
    if (temp->temp == 0) {
        return ESP_FAIL;
    }
    PoolstateElemTempTyp const elem_temp_typ = static_cast<PoolstateElemTempTyp>(typ);
    
    switch (elem_temp_typ) {
        case PoolstateElemTempTyp::TEMP:            
            _alloc_uint(value, temp->temp);
            break;
        default:
            ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t
_thermostat(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * const value)
{
    poolstate_thermo_t const * const thermostat = &state->thermos[idx];
    poolstate_elem_thermos_typ_t const elem_thermo_typ = static_cast<poolstate_elem_thermos_typ_t>(typ);

    switch (elem_thermo_typ) {
        case poolstate_elem_thermos_typ_t::TEMP:
            _alloc_uint(value, thermostat->temp);
            break;
        case poolstate_elem_thermos_typ_t::SET_POINT:
            _alloc_uint(value, thermostat->set_point);
            break;
        case poolstate_elem_thermos_typ_t::HEAT_SRC:
            _alloc_str(value, network_heat_src_str(static_cast<network_heat_src_t>(thermostat->heat_src)));
            break;
        case poolstate_elem_thermos_typ_t::HEATING:
            _alloc_bool(value, thermostat->heating);
            break;
        default:
            ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t
_schedule(poolstate_t const * const state, uint8_t const typ_dummy, uint8_t const idx, poolstate_get_value_t * const value)
{
    (void)typ_dummy;
    network_pool_circuit_t const circuit = (network_pool_circuit_t)idx;
    poolstate_sched_t const * const sched = &state->scheds[static_cast<uint8_t>(idx)];

    if (sched->active) {
        _alloc_strs(value, 
                    network_ctrl_time_str(sched->start / 60, sched->start % 60),
                    network_ctrl_time_str(sched->stop / 60, sched->stop % 60));
    } else {
        _alloc_str(value, "no sched");
    }
    return ESP_OK;
}

static esp_err_t
_pump(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * const value)
{
    poolstate_pump_t const * const pump = &state->pump;
    poolstate_elem_pump_typ_t const elem_pump_typ = static_cast<poolstate_elem_pump_typ_t>(typ);

    switch (elem_pump_typ) {
        case poolstate_elem_pump_typ_t::MODE:
            _alloc_str(value, network_pump_mode_str(static_cast<network_pump_mode_t>(pump->mode)));
            break;
        case poolstate_elem_pump_typ_t::RUNNING:
            _alloc_bool(value, pump->running);
            break;
        case poolstate_elem_pump_typ_t::STATE:
            _alloc_str(value, network_pump_state_str(static_cast<network_pump_state_t>(pump->state)));
            break;
        case poolstate_elem_pump_typ_t::PWR:
            _alloc_uint(value, pump->pwr);
            break;
        case poolstate_elem_pump_typ_t::GPM:
            _alloc_uint(value, pump->gpm);
            break;
        case poolstate_elem_pump_typ_t::RPM:
            _alloc_uint(value, pump->rpm);
            break;
        case poolstate_elem_pump_typ_t::PCT:
            _alloc_uint(value, pump->pct);
            break;
        case poolstate_elem_pump_typ_t::ERR:
            _alloc_uint(value, pump->err);
            break;
        case poolstate_elem_pump_typ_t::TIMER:
            _alloc_uint(value, pump->timer);
            break;
        default:
            ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * chlor
 **/

static esp_err_t
_chlor(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * const value)
{
    poolstate_chlor_t const * const chlor = &state->chlor;
    poolstate_elem_chlor_typ_t const elem_chlor_typ = static_cast<poolstate_elem_chlor_typ_t>(typ);

    switch (elem_chlor_typ) {
        case poolstate_elem_chlor_typ_t::NAME:
            _alloc_str(value, chlor->name);
            break;
        case poolstate_elem_chlor_typ_t::PCT:
            _alloc_uint(value, chlor->pct);
            break;
        case poolstate_elem_chlor_typ_t::SALT:
            _alloc_uint(value, chlor->salt);
            break;
        case poolstate_elem_chlor_typ_t::STATUS:
            _alloc_str(value, poolstate_str_chlor_status_str(chlor->status));
            break;
        default:
            ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            return ESP_FAIL;
    }
    return ESP_OK;
}


/**
 * modes
 **/

static esp_err_t
_modes(poolstate_t const * const state, uint8_t const typ, uint8_t const idx, poolstate_get_value_t * const value)
{
    poolstate_modes_t const * const modes = &state->modes;
    poolstate_elem_modes_typ_t const elem_modes_typ = static_cast<poolstate_elem_modes_typ_t>(typ);

    switch (elem_modes_typ) {
        case poolstate_elem_modes_typ_t::SERVICE:
            _alloc_bool(value, modes->set[static_cast<uint8_t>(network_pool_mode_t::SERVICE)]);
            break;
        case poolstate_elem_modes_typ_t::TEMP_INC:
            _alloc_bool(value, modes->set[static_cast<uint8_t>(network_pool_mode_t::TEMP_INC)]);
            break;
        case poolstate_elem_modes_typ_t::FREEZE_PROT:
            _alloc_bool(value, modes->set[static_cast<uint8_t>(network_pool_mode_t::FREEZE_PROT)]);
            break;
        case poolstate_elem_modes_typ_t::TIMEOUT:
            _alloc_bool(value, modes->set[static_cast<uint8_t>(network_pool_mode_t::TIMEOUT)]);
            break;
        default:
            if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN) {
                ESP_LOGW(TAG, "%s unknown sub_typ(%u)", __func__, typ);
            } 
            return ESP_FAIL;
    }
    return ESP_OK;
}


/**
 * all together now
 **/

typedef esp_err_t (* dispatch_fnc_t)(poolstate_t const * const state, uint8_t const sub_typ, uint8_t const idx, poolstate_get_value_t * const value);

typedef struct dispatch_t {
    poolstate_elem_typ_t const  typ;
    dispatch_fnc_t              fnc;
} dispatch_t;

static dispatch_t const _dispatches[] = {
    { poolstate_elem_typ_t::SYSTEM, _system},
    { poolstate_elem_typ_t::TEMP,   _temp},
    { poolstate_elem_typ_t::THERMO, _thermostat},
    { poolstate_elem_typ_t::SCHED,  _schedule},
    { poolstate_elem_typ_t::PUMP,   _pump},
    { poolstate_elem_typ_t::CHLOR,  _chlor},
    { poolstate_elem_typ_t::MODES,  _modes},
};

esp_err_t
OpnPoolState::get_poolstate_value(poolstate_t const * const state, poolstate_get_params_t const * const params, poolstate_get_value_t * const value)
{
    dispatch_t const * dispatch = _dispatches;
    for (uint8_t ii = 0; ii < ARRAY_SIZE(_dispatches); ii++, dispatch++) {
        if (params->elem_typ == dispatch->typ) {

            return dispatch->fnc(state, params->elem_sub_typ, params->idx, value);  // caller MUST free *value
        }
    }
    return ESP_FAIL;
}

}  // namespace opnpool
}  // namespace esphome