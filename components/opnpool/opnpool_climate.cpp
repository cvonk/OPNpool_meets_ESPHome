/**
 * @file opnpool_climate.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Actuates climate settings from Home Assistant on the pool controller.
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
 * 
 * @details
 * Implements the climate entity interface for the OPNpool component, allowing ESPHome to
 * control and monitor pool/spa heating via RS-485. Handles mapping between ESPHome
 * climate calls and pool controller commands, and updates climate state from pool
 * controller feedback.
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
#include <type_traits>

#include "to_str.h"
#include "enum_helpers.h"
#include "opnpool_climate.h"  // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpool_state.h"
#include "opnpool_switch.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_climate";

    // helper to convert enum class to its underlying type
template<typename E>
constexpr auto to_index(E e) -> typename std::underlying_type<E>::type {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

inline float fahrenheit_to_celsius(float f) {
    return (f - 32.0f) * 5.0f / 9.0f;
}

inline float celsius_to_fahrenheit(float c) {
    return c * 9.0f / 5.0f + 32.0f;
}

enum class custom_presets_t {
    NONE,
    Heat,
    SolarPreferred,
    Solar
};

void OpnPoolClimate::setup()
{
    // nothing to do here - my parent takes care of me :-)
}

void OpnPoolClimate::dump_config()
{
    LOG_CLIMATE("  ", "Climate", this);
}


/**
 * @brief Get the climate traits for the OPNpool climate entity
 * 
 * @details
 * Defines the supported modes, temperature range, and custom presets for the OPNpool
 * climate entity. Specifies how the pool/spa heating interface appears to Home Assistant,
 * including available climate modes, valid temperature range, and selectable heating
 * sources.
 * Home Assistant will use the custom presets (Heat/SolarPreferred/Solar), but has to use
 * the regular preset (NONE) to disable the heating source.
 * 
 * @return climate::ClimateTraits 
 */
climate::ClimateTraits OpnPoolClimate::traits()
{
    auto traits = climate::ClimateTraits();  // see climate_traits.h
    
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_HEAT
    });
    traits.set_visual_min_temperature(0);   // 32°F in Celsius
    traits.set_visual_max_temperature(43);  // 110°F in Celsius
    traits.set_visual_temperature_step(1);

        // support for disabling the heating source
    traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);

        // custom heat sources
    traits.set_supported_custom_presets({
        enum_str(custom_presets_t::Heat),
        enum_str(custom_presets_t::SolarPreferred),
        enum_str(custom_presets_t::Solar)
    });
    return traits;
}


/**
 * @brief
 * Handles requests from Home Assistant to change the climate state of the pool or spa.
 *
 * @details
 * Processes incoming commands for target temperature, climate mode (OFF/HEAT), and
 * heat source (custom preset).
 * The pool controller doesn't have a concept of climate modes.  Instead we map it to
 * the pool/spa circuit switch.
 * Constructs and sends protocol messages to the pool controller if thermostat
 * settings have changed.
 * Unsupported climate modes are logged and reported to Home Assistant as OFF.
 * State is not published immediately; updates are sent after confirmation from the
 * pool controller.
 * 
 * @param call The climate call object containing requested changes from Home Assistant.
 */
void OpnPoolClimate::control(const climate::ClimateCall &call)
{
    uint8_t const climate_idx = this->get_idx();
    uint8_t const climate_pool_idx = to_index(ClimateId::POOL_CLIMATE);
    uint8_t const climate_spa_idx = to_index(ClimateId::SPA_CLIMATE);

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
        float const target_temp_fahrenheit = celsius_to_fahrenheit(target_temp_celsius);
        ESP_LOGV(TAG, "HA requests target temperature [%u] to %.1f°F", climate_idx, target_temp_fahrenheit);

        thermos_new[climate_idx].set_point = static_cast<uint8_t>(target_temp_fahrenheit);
    }

        // handle mode changes (OFF, HEAT, AUTO) by turning the POOL or SPA circuit on or off

    if (call.get_mode().has_value()) {

        climate::ClimateMode const requested_mode = *call.get_mode();
        
        ESP_LOGV(TAG, "HA requests climate mode [%u] to mode=%u", climate_idx, static_cast<int>(requested_mode));
        uint8_t switch_idx = (climate_idx == climate_pool_idx) 
                           ? to_index(SwitchId::POOL)
                           : to_index(SwitchId::SPA);
        
        switch (requested_mode) {
            case climate::CLIMATE_MODE_OFF:  // mode 0
                ESP_LOGVV(TAG, "Turning off switch[%u]", switch_idx);
                parent_->get_switch(switch_idx)->write_state(false);
                break;                
            case climate::CLIMATE_MODE_HEAT:  // mode 3
                ESP_LOGVV(TAG, "Turning on switch[%u]", switch_idx);
                parent_->get_switch(switch_idx)->write_state(true);
                break;
            default:
                ESP_LOGW(TAG, "Unsupported climate mode: %d", static_cast<int>(requested_mode));
                this->mode = climate::CLIMATE_MODE_OFF;
                this->action = climate::CLIMATE_ACTION_OFF;
                this->publish_state();
                break;
        }
    }

        // handle heat source changes (based on custom preset)

    char const * preset_str = call.get_custom_preset();
    if (preset_str != nullptr) {

        ESP_LOGV(TAG, "HA requests heat source [%u] change to %s", climate_idx, preset_str);

        if (strcasecmp(preset_str, enum_str(custom_presets_t::Heat)) == 0) {
            thermos_new[climate_idx].heat_src = to_index(network_heat_src_t::HEATER);
        } else if (strcasecmp(preset_str, enum_str(custom_presets_t::SolarPreferred)) == 0) {
            thermos_new[climate_idx].heat_src = to_index(network_heat_src_t::SOLAR_PREF);
        } else if (strcasecmp(preset_str, enum_str(custom_presets_t::Solar)) == 0) {
            thermos_new[climate_idx].heat_src = to_index(network_heat_src_t::SOLAR);
        }
    } else if (call.get_preset().has_value()) {

        climate::ClimatePreset new_preset = *call.get_preset();
        
        if (new_preset == climate::CLIMATE_PRESET_NONE) {
            ESP_LOGV(TAG, "HA requests heat source [%u] change to %u(NONE)", climate_idx, new_preset);
            thermos_new[climate_idx].heat_src = to_index(network_heat_src_t::NONE);
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


/**
 * @brief
 * Updates the climate entity state from the latest pool controller feedback.
 *
 * @details
 * Extracts current and target temperatures, climate mode, heat source, and heating action
 * from the provided pool state.
 * Maps pool controller heat source values to custom presets for Home Assistant.
 * Determines the climate action (heating, idle, or off) based on controller feedback.
 * Publishes the updated state to Home Assistant only if any relevant value has changed.
 *
 * @param new_state Pointer to the latest poolstate_t structure containing updated pool controller data.
 */
void
OpnPoolClimate::update_climate(const poolstate_t * new_state) 
{    
    uint8_t const climate_idx = this->get_idx();
    uint8_t const climate_pool_idx = to_index(ClimateId::POOL_CLIMATE);
    uint8_t const climate_spa_idx = to_index(ClimateId::SPA_CLIMATE);

        // update temperatures

    poolstate_thermo_t const * const thermo = &new_state->thermos[climate_idx];

    float const new_current_temp_in_fahrenheit = new_state->temps[to_index(poolstate_temp_typ_t::WATER)].temp;
    float const new_current_temp = fahrenheit_to_celsius(new_current_temp_in_fahrenheit);
    float const new_target_temp_in_fahrenheit = thermo->set_point;
    float new_target_temp = fahrenheit_to_celsius(new_target_temp_in_fahrenheit);
    
        // update mode based on {circuit state, heating status}

    uint8_t switch_idx = (climate_idx == climate_pool_idx) 
                        ? to_index(SwitchId::POOL)
                        : to_index(SwitchId::SPA);

    climate::ClimateMode new_mode;

    if (new_state->circuits.active[switch_idx]) {
        new_mode = climate::CLIMATE_MODE_HEAT;
    } else {
        new_mode = climate::CLIMATE_MODE_OFF;
    }
    
        // update custom preset (based on heat source)

    char const * new_custom_preset = enum_str(static_cast<custom_presets_t>(thermo->heat_src));
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

    ESP_LOGVV(TAG, "RX updated climate[%u]: current=%.0f, target=%.0f, mode=%u, custom_preset=%s, action=%u", climate_idx, new_current_temp, new_target_temp, static_cast<int8_t>(new_mode), new_custom_preset, to_index(new_action));

    this->publish_value_if_changed(climate_idx, new_current_temp, new_target_temp,
                                  new_mode, new_custom_preset, new_action);
}

/**
 * @brief
 * Publishes the climate entity state to Home Assistant if any relevant value has changed.
 *
 * @details
 * Compares the new climate state (current temperature, target temperature, mode, custom preset, 
 * and action) with the last published state. If any value differs, updates the internal state,
 * sets the appropriate preset or custom preset, and publishes the new state to Home Assistant.
 * This avoids redundant updates to Home Assistant.
 *
 * @param idx                        Index of the climate entity (pool or spa). (diagnostic only)
 * @param value_current_temperature  The updated current temperature in Celsius.
 * @param value_target_temperature   The updated target temperature in Celsius.
 * @param value_mode                 The updated climate mode (OFF/HEAT).
 * @param value_custom_preset        The updated custom preset string (heat source).
 * @param value_action               The updated climate action (heating, idle, or off).
 */
void 
OpnPoolClimate::publish_value_if_changed(uint8_t const idx, float const value_current_temperature, float const value_target_temperature, climate::ClimateMode const value_mode, char const * value_custom_preset, climate::ClimateAction const value_action)
{
    last_value_t * const last = &last_value_;

    if (!last->valid ||
        last->current_temp != value_current_temperature ||
        last->target_temp != value_target_temperature ||
        last->mode != value_mode ||
        strcasecmp(last->custom_preset, value_custom_preset) != 0 ||
        last->action != value_action) {
        
        this->current_temperature = value_current_temperature;
        this->target_temperature = value_target_temperature;
        this->mode = value_mode;
        this->action = value_action;

            // NONE not handled by the regular preset, not a custom preset
        if (strcasecmp(value_custom_preset, enum_str(custom_presets_t::NONE)) == 0) {
            ESP_LOGV(TAG, "Setting climate[%u] preset to NONE", idx);
            set_preset_(climate::CLIMATE_PRESET_NONE);
            clear_custom_preset_();
        } else {
            ESP_LOGV(TAG, "Setting climate[%u] custom_preset to %s", idx, value_custom_preset);
            this->set_custom_preset_(value_custom_preset);
        }

        ESP_LOGV(TAG, "Publishing climate[%u]: current=%.0f, target=%.0f, mode=%u, *preset=%s, action=%u", idx, value_current_temperature, value_target_temperature, static_cast<int8_t>(value_mode), value_custom_preset, to_index(value_action));

        this->publish_state();
            
        *last = {
            .valid = true,
            .current_temp = value_current_temperature,
            .target_temp = value_target_temperature,
            .custom_preset = value_custom_preset,
            .mode = value_mode,
            .action = value_action,
        };
    }
}

}  // namespace opnpool
}  // namespace esphome