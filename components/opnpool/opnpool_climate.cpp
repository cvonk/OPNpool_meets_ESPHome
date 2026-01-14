/**
 * @file opnpool_climate.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Pool climate interface
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
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
 * SPDX-FileCopyrightText: Copyright 2026 Coert Vonk
 */

#include <esp_timer.h>
#include <esphome/core/log.h>

#include "opnpool_climate.h"  // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpool.h"
#include "opnpool_state.h"
#include "opnpool_switch.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_climate";

void OpnPoolClimate::setup() {
    // Nothing to do here - parent handles setup
}

void OpnPoolClimate::dump_config() {
    LOG_CLIMATE("  ", "Climate", this);
}

enum class custom_presets_t {
    NONE,
    Heat,
    SolarPreferred,
    Solar
};

inline const char *
custom_presets_str(custom_presets_t const preset)
{
    auto name = magic_enum::enum_name(preset);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[3];
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(preset));
    return buf;
}


climate::ClimateTraits OpnPoolClimate::traits() {

    auto traits = climate::ClimateTraits();  // see climate_traits.h
    
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_HEAT
    });
    traits.set_visual_min_temperature(0);   // 32°F in Celsius
    traits.set_visual_max_temperature(43);  // 110°F in Celsius
    traits.set_visual_temperature_step(1);

        // support for clearing presets
    traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);

        // custom heating presets
    traits.set_supported_custom_presets({
        custom_presets_str(custom_presets_t::Heat),
        custom_presets_str(custom_presets_t::SolarPreferred),
        custom_presets_str(custom_presets_t::Solar)
    });
    return traits;
}

void OpnPoolClimate::control(const climate::ClimateCall &call) {
    
    uint8_t const climate_idx = this->get_idx();
    uint8_t const climate_pool_idx = static_cast<uint8_t>(ClimateId::POOL);
    uint8_t const climate_spa_idx = static_cast<uint8_t>(ClimateId::SPA);

       // get both thermostats ('cause the change request needs to references them both)

    poolstate_t state;
    parent_->get_opnpool_state()->get(&state);
    poolstate_thermo_t thermos_old[POOLSTATE_THERMO_TYP_COUNT];
    poolstate_thermo_t thermos_new[POOLSTATE_THERMO_TYP_COUNT];
    memcpy(thermos_old, state.thermos, sizeof(thermos_old));
    memcpy(thermos_new, state.thermos, sizeof(thermos_new));

        // handle target temperature changes

    if (call.get_target_temperature().has_value()) {

        float const target_temp_celsius = *call.get_target_temperature();
        float const target_temp_fahrenheit = target_temp_celsius * 9.0f / 5.0f + 32.0f;
        ESP_LOGV(TAG, "HA requests target temperature [%u] to %.1f°F", climate_idx, target_temp_fahrenheit);

        thermos_new[climate_idx].set_point = static_cast<uint8_t>(target_temp_fahrenheit);
    }

        // handle mode changes (OFF, HEAT, AUTO) by turning the POOL or SPA circuit on or off

    if (call.get_mode().has_value()) {

        climate::ClimateMode const requested_mode = *call.get_mode();
        
        ESP_LOGV(TAG, "HA requests climate mode [%u] to mode=%u", climate_idx, static_cast<int>(requested_mode));
        uint8_t switch_idx = (climate_idx == climate_pool_idx) 
                           ? static_cast<uint8_t>(SwitchId::POOL)
                           : static_cast<uint8_t>(SwitchId::SPA);
        
        switch (requested_mode) {
            case climate::CLIMATE_MODE_OFF:  // mode 0
                ESP_LOGVV(TAG, "Turning off switch[%u]", switch_idx);
                parent_->get_switch(switch_idx)->on_switch_command(false);
                break;                
            case climate::CLIMATE_MODE_HEAT:  // mode 3
                ESP_LOGVV(TAG, "Turning on switch[%u]", switch_idx);
                parent_->get_switch(switch_idx)->on_switch_command(true);
                break;
            default:
                ESP_LOGW(TAG, "Unsupported climate mode: %d", static_cast<int>(requested_mode));
                break;
        }
    }

        // handle heat source changes (based on custom preset)

    const char* preset_str = call.get_custom_preset();
    if (preset_str != nullptr) {

        ESP_LOGV(TAG, "HA requests heat source [%u] change to %s", climate_idx, preset_str);

        if (strcasecmp(preset_str, custom_presets_str(custom_presets_t::Heat)) == 0) {
            thermos_new[climate_idx].heat_src = static_cast<uint8_t>(network_heat_src_t::HEATER);
        } else if (strcasecmp(preset_str, custom_presets_str(custom_presets_t::SolarPreferred)) == 0) {
            thermos_new[climate_idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR_PREF);
        } else if (strcasecmp(preset_str, custom_presets_str(custom_presets_t::Solar)) == 0) {
            thermos_new[climate_idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR);
        }
    } else if (call.get_preset().has_value()) {
        climate::ClimatePreset new_preset = *call.get_preset();
        
        if (new_preset == climate::CLIMATE_PRESET_NONE) {
            ESP_LOGV(TAG, "HA requests heat source [%u] change to %u(NONE)", climate_idx, new_preset);
            thermos_new[climate_idx].heat_src = static_cast<uint8_t>(network_heat_src_t::NONE);
        }
    }        

        // actuate thermostat changes if necessary

    bool const thermos_changed = memcmp(thermos_old, thermos_new, sizeof(thermos_old)) != 0;

    if (thermos_changed) {

        uint8_t const pool_heat_src = thermos_new[climate_pool_idx].heat_src;
        uint8_t const spa_heat_src = thermos_new[climate_spa_idx].heat_src;

        network_msg_t msg = {
            .typ = network_msg_typ_t::CTRL_HEAT_SET,
            .u = {
                .ctrl_heat_set = {
                    .poolSetpoint = thermos_new[climate_pool_idx].set_point,
                    .spaSetpoint = thermos_new[climate_spa_idx].set_point,
                    .heatSrc = static_cast<uint8_t>(pool_heat_src | (spa_heat_src << 2))
                },
            },
        };

        ESP_LOGV(TAG, "Sending HEAT_SET: pool_sp=%u°F, spa_sp=%u°F, heat_src=0x%02X", 
                  msg.u.ctrl_heat_set.poolSetpoint, 
                  msg.u.ctrl_heat_set.spaSetpoint,
                  msg.u.ctrl_heat_set.heatSrc);
        ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc());
    }

    // DON'T publish state here - wait for pool controller confirmation
}

void
OpnPoolClimate::update_climate(const poolstate_t * new_state) 
{    
    uint8_t const climate_idx = this->get_idx();
    uint8_t const climate_pool_idx = static_cast<uint8_t>(ClimateId::POOL);
    uint8_t const climate_spa_idx = static_cast<uint8_t>(ClimateId::SPA);

        // update temperatures

    poolstate_thermo_t const * const thermo = &new_state->thermos[climate_idx];

    float const new_current_temp_in_fahrenheit = new_state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
    float const new_current_temp = (new_current_temp_in_fahrenheit - 32.0f) * 5.0f / 9.0f;

    float new_target_temp_in_fahrenheit = thermo->set_point;
    float new_target_temp = (new_target_temp_in_fahrenheit - 32.0f) * 5.0f / 9.0f;
    
        // update mode based on {circuit state, heating status}

    uint8_t switch_idx = (climate_idx == climate_pool_idx) 
                        ? static_cast<uint8_t>(SwitchId::POOL)
                        : static_cast<uint8_t>(SwitchId::SPA);

    climate::ClimateMode new_mode;

    if (new_state->circuits.active[switch_idx]) {
        new_mode = climate::CLIMATE_MODE_HEAT;
    } else {
        new_mode = climate::CLIMATE_MODE_OFF;
    }
    
        // update custom preset (based on heat source)

    const char * new_custom_preset = nullptr;
    switch (static_cast<network_heat_src_t>(thermo->heat_src)) {
        case network_heat_src_t::NONE:
            new_custom_preset = custom_presets_str(custom_presets_t::NONE);
            break;
        case network_heat_src_t::HEATER:
            new_custom_preset = custom_presets_str(custom_presets_t::Heat);
            break;
        case network_heat_src_t::SOLAR_PREF:
            new_custom_preset = custom_presets_str(custom_presets_t::SolarPreferred);
            break;
        case network_heat_src_t::SOLAR:
            new_custom_preset = custom_presets_str(custom_presets_t::Solar);
            break;
        default:
            ESP_LOGW(TAG, "Unknown heat source: %u", thermo->heat_src);
            break;
    }
    ESP_LOGVV(TAG, "Mapped heat source [%u]: %u(%s)", climate_idx, thermo->heat_src, new_custom_preset);

        // update action

    climate::ClimateAction new_action;

    if (thermo->heating) {
        new_action = climate::CLIMATE_ACTION_HEATING;
    } else if (new_mode == climate::CLIMATE_MODE_OFF) {
        new_action = climate::CLIMATE_ACTION_OFF;
    } else {
        new_action = climate::CLIMATE_ACTION_IDLE;
    }

    ESP_LOGVV(TAG, "RX updated climate[%u]: current=%.0f, target=%.0f, mode=%u, custom_preset=%s, action=%u", climate_idx, new_current_temp, new_target_temp, static_cast<int8_t>(new_mode), new_custom_preset, static_cast<uint8_t>(new_action));

    this->publish_state_if_changed(climate_idx, new_current_temp, new_target_temp,
                                  new_mode, new_custom_preset, new_action);
}


void OpnPoolClimate::publish_state_if_changed(uint8_t const idx, float const new_current_temperature, float const new_target_temperature, climate::ClimateMode const new_mode, char const * new_custom_preset, climate::ClimateAction const new_action) {

    last_state_t * last = &this->last_state_;

    if (!last->valid ||
        last->current_temp != new_current_temperature ||
        last->target_temp != new_target_temperature ||
        last->mode != new_mode ||
        strcasecmp(last->custom_preset, new_custom_preset) != 0 ||
        last->action != new_action) {
        
        this->current_temperature = new_current_temperature;
        this->target_temperature = new_target_temperature;
        this->mode = new_mode;
        this->action = new_action;

        if (strcasecmp(new_custom_preset, custom_presets_str(custom_presets_t::NONE)) == 0) {
            ESP_LOGV(TAG, "Setting climate[%u] preset to NONE", idx);
            set_preset_(climate::CLIMATE_PRESET_NONE);
            clear_custom_preset_();
        } else {
            ESP_LOGV(TAG, "Setting climate[%u] custom_preset to %s", idx, new_custom_preset);
            this->set_custom_preset_(new_custom_preset);
        }

        ESP_LOGV(TAG, "Publishing climate[%u]: current=%.0f, target=%.0f, mode=%u, *preset=%s, action=%u", idx, new_current_temperature, new_target_temperature, static_cast<int8_t>(new_mode), new_custom_preset, static_cast<uint8_t>(new_action));

        this->publish_state();
            
        last->current_temp = new_current_temperature;
        last->target_temp = new_target_temperature;
        last->mode = new_mode;
        last->custom_preset = new_custom_preset;
        last->action = new_action;
        last->valid = true;
    }
}

}  // namespace opnpool
}  // namespace esphome