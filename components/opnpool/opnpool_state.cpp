/**
 * @file opnpool_state.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - maintains the state information for the pool
 * 
 * @copyright 2014, 2019, 2022, 2026, Coert Vonk
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

#include <esphome/core/log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "opnpool_state.h"
#include "opnpool.h"
#include "ipc.h"
#include "network.h"

namespace esphome {

namespace opnpool {

static char const * const TAG = "opnpool_state";

OpnPoolState::OpnPoolState(OpnPool *parent) : parent_(parent) {
    protected_.xMutex = xSemaphoreCreateMutex();
    protected_.state = new poolstate_t();
    protected_.valid = false;
    
    if (protected_.xMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    protected_.valid = true;
}

void OpnPoolState::set(poolstate_t const * const state) {
    if (xSemaphoreTake(protected_.xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(protected_.state, state, sizeof(poolstate_t));
        protected_.valid = true;
        xSemaphoreGive(protected_.xMutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex in set()");
    }
}

esp_err_t OpnPoolState::get(poolstate_t * const state) {
    if (xSemaphoreTake(protected_.xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (protected_.valid) {
            memcpy(state, protected_.state, sizeof(poolstate_t));
        }
        xSemaphoreGive(protected_.xMutex);
        return protected_.valid ? ESP_OK : ESP_ERR_INVALID_STATE;
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex in get()");
        return ESP_ERR_TIMEOUT;
    }
}

#if 0
bool
OpnPoolState::get_circuit(char const * const key)
{
	uint16_t mask = 0x00001;
	for (uint_least8_t ii = 0; mask; ii++) {
		if (strcmp(key, network_pool_circuit_str(ii + 1)) == 0) {
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
	current.pool.setPoint = this->protected_.state->thermos[poolstate_thermo_type_t::pool].setPoint;
	current.pool.heatSrc = this->protected_.state->thermos[poolstate_thermo_type_t::pool].heatSrc;
	current.spa.setPoint = this->protected_.state->thermos[poolstate_thermo_type_t::spa].setPoint;
	current.spa.heatSrc = this->protected_.state->thermos[poolstate_thermo_type_t::spa].heatSrc;
	return current;
}

char const *
OpnPoolState::get_heat_src(char const * const key)
{
	if (strcmp(key, "pool") == 0) {
		return _heatSrcStr(this->protected_.state->thermos[poolstate_thermo_type_t::pool].heatSrc);
	}
	if (strcmp(key, "spa") == 0) {
		return _heatSrcStr(this->protected_.state->thermos[poolstate_thermo_type_t::spa].heatSrc);
	}
	return "err";
}

uint8_t
OpnPoolState::get_heat_sp(char const * const key)
{
	if (strcmp(key, "pool") == 0) {
		return this->protected_.state->thermos[poolstate_thermo_type_t::pool].setPoint;
	}
	if (strcmp(key, "spa") == 0) {
		return this->protected_.state->thermos[poolstate_thermo_type_t::spa].setPoint;
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