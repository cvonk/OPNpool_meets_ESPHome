/**
 * @brief OPNpool - maintains the state information for the pool
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esphome/core/log.h>
//#include <cstdlib>

#include "network.h"
#include "opnpoolstate.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpoolstate";

OpnPoolState::OpnPoolState(OpnPool * parent) : parent_(parent)
{
    protected_.xMutex = xSemaphoreCreateMutex();
    if (protected_.xMutex == nullptr) {
        ESP_LOGE(TAG, "mutex creation failed");
        protected_.valid = false;
        protected_.state = nullptr;
        return;
    }
    
    protected_.state = static_cast<poolstate_t *>(calloc(1, sizeof(poolstate_t)));
    if (protected_.state == nullptr) {
        ESP_LOGE(TAG, "allocation failed");
        protected_.valid = false;
        return;
    }
    
    protected_.state->chlor.name[0] = '\0';
    protected_.valid = true;
}

void
OpnPoolState::set(poolstate_t const * const state)
{
    xSemaphoreTake( this->protected_.xMutex, portMAX_DELAY );
    {
        this->protected_.valid = true;
        memcpy(this->protected_.state, state, sizeof(poolstate_t));
    }
    xSemaphoreGive( this->protected_.xMutex );
}

esp_err_t
OpnPoolState::get(poolstate_t * const state)
{
    bool valid;
    xSemaphoreTake( this->protected_.xMutex, portMAX_DELAY );
    {
        valid = this->protected_.valid;
        memcpy(state, this->protected_.state, sizeof(poolstate_t));
    }
    xSemaphoreGive( this->protected_.xMutex );
    return valid ? ESP_OK : ESP_FAIL;
}

#if 0
bool
OpnPoolState::get_circuit(char const * const key)
{
	uint16_t mask = 0x00001;
	for (uint_least8_t ii = 0; mask; ii++) {
		if (strcmp(key, network_circuit_str(ii + 1)) == 0) {
			return value & mask;
		}
		mask <<= 1;
	}
	return false;
}

heatpoolstate_t
OpnPoolState::get_heat(void)
{
	heatpoolstate_t current;
	current.pool.setPoint = this->protected_.state->thermos[POOLSTATE_THERMO_TYP_pool].setPoint;
	current.pool.heatSrc = this->protected_.state->thermos[POOLSTATE_THERMO_TYP_pool].heatSrc;
	current.spa.setPoint = this->protected_.state->thermos[POOLSTATE_THERMO_TYP_spa].setPoint;
	current.spa.heatSrc = this->protected_.state->thermos[POOLSTATE_THERMO_TYP_spa].heatSrc;
	return current;
}

char const *
OpnPoolState::get_heat_src(char const * const key)
{
	if (strcmp(key, "pool") == 0) {
		return _heatSrcStr(this->protected_.state->thermos[POOLSTATE_THERMO_TYP_pool].heatSrc);
	}
	if (strcmp(key, "spa") == 0) {
		return _heatSrcStr(this->protected_.state->thermos[POOLSTATE_THERMO_TYP_spa].heatSrc);
	}
	return "err";
}

uint8_t
OpnPoolState::get_heat_sp(char const * const key)
{
	if (strcmp(key, "pool") == 0) {
		return this->protected_.state->thermos[POOLSTATE_THERMO_TYP_pool].setPoint;
	}
	if (strcmp(key, "spa") == 0) {
		return this->protected_.state->thermos[POOLSTATE_THERMO_TYP_spa].setPoint;
	}
	return 0;
}

void
OpnPoolState::name_schedule(JsonObject & json, uint8_t const circuit, uint16_t const start, uint16_t const stop)
{
	char const * const key = _circuitName(circuit);
	JsonObject & sched = json.createNestedObject(key);
	sched["start"] = Utils::strTime(start / 60, start % 60);
	sched["stop"] = Utils::strTime(stop / 60, stop % 60);
}

#endif

}  // namespace opnpool
}  // namespace esphome