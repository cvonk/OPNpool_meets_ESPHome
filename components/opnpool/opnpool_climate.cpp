#include <esp_timer.h>
#include <esphome/core/log.h>

#include "opnpool_climate.h"  // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpoolstate.h"

namespace esphome {
namespace opnpool {

static const char *TAG = "opnpool.climate";

void OpnPoolClimate::setup() {
    // Nothing to do here - parent handles setup
}

void OpnPoolClimate::dump_config() {
    LOG_CLIMATE("  ", "Climate", this);
}

void OpnPoolClimate::publish_state_if_changed() {
    if (!last_state_valid_ ||
        last_current_temp_ != this->current_temperature ||
        last_target_temp_ != this->target_temperature ||
        last_mode_ != this->mode ||
        last_action_ != this->action) {
        
        this->publish_state();
        last_current_temp_ = this->current_temperature;
        last_target_temp_ = this->target_temperature;
        last_mode_ = this->mode;
        last_action_ = this->action;
        last_state_valid_ = true;
    }
}

climate::ClimateTraits OpnPoolClimate::traits() {
    auto traits = climate::ClimateTraits();
    
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_AUTO
    });
    
    traits.set_visual_min_temperature(0);   // 32°F in Celsius
    traits.set_visual_max_temperature(43);  // 110°F in Celsius
    traits.set_visual_temperature_step(0.5);
    
    // Custom presets for heat source selection
    traits.set_supported_custom_presets({
        "None",
        "Heat",
        "Solar Preferred",
        "Solar"
    });
    
    // Target temperature and current temperature are enabled by default
    // when modes are set and visual temperature range is configured
    
    return traits;
}

void OpnPoolClimate::control(const climate::ClimateCall &call) {
    
    uint8_t const idx = this->parent_->get_climate_index(this);

    uint8_t const pool_idx = static_cast<uint8_t>(network_pool_circuit_t::POOL);
    uint8_t const spa_idx = static_cast<uint8_t>(network_pool_circuit_t::SPA);

       // get thermostat settings (we need the current settings, as we may only change one of them)

    poolstate_thermo_t thermos_old[POOLSTATE_THERMO_TYP_COUNT];
    poolstate_thermo_t thermos_new[POOLSTATE_THERMO_TYP_COUNT];
    {
        poolstate_t state;
        parent_->get_opnpool_state()->get(&state);
        memcpy(thermos_old, state.thermos, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT);
        memcpy(thermos_new, state.thermos, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT);
    }

        // handle mode changes (OFF, HEAT, AUTO)

    if (call.get_mode().has_value()) {
        climate::ClimateMode mode = *call.get_mode();
        
        ESP_LOGV(TAG, "Climate mode changed: idx=%u to mode=%d", idx, static_cast<int>(mode));
        
            // map climate index to circuit index
        uint8_t idx = idx == pool_idx ? static_cast<uint8_t>(network_pool_circuit_t::POOL) : static_cast<uint8_t>(network_pool_circuit_t::SPA);
        
        switch (mode) {
            case climate::CLIMATE_MODE_OFF:
                // 2BD: this will need to be done through opnpool_switch.cpp
                //this->parent_->on_switch_command(idx, false);
                break;
                
            case climate::CLIMATE_MODE_HEAT:
            case climate::CLIMATE_MODE_AUTO:
                // 2BD: this will need to be done through opnpool_switch.cpp
                //this->parent_->on_switch_command(idx, true);
                break;
                
            default:
                ESP_LOGW(TAG, "Unsupported climate mode: %d", static_cast<int>(mode));
                break;
        }
    }

        // handle target temperature changes

    if (call.get_target_temperature().has_value()) {
        float target_temp_celsius = *call.get_target_temperature();
        
            // convert to Fahrenheit (ESPHome uses Celsius internally)
        float target_temp_fahrenheit = target_temp_celsius * 9.0f / 5.0f + 32.0f;
        
        ESP_LOGV(TAG, "Target temperature changed: idx=%u to %.1f°F", idx, target_temp_fahrenheit);

        thermos_new[idx].set_point = static_cast<uint8_t>(target_temp_fahrenheit);
    }

        // handle heat source changes

    const char* preset_str = call.get_custom_preset();
    if (preset_str != nullptr) {

        ESP_LOGV(TAG, "Heat source changed: idx=%u to %s", idx, preset_str);

        if (strcasecmp(preset_str, "None") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::NONE);
        } else if (strcasecmp(preset_str, "Heat") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::HEATER);
        } else if (strcasecmp(preset_str, "Solar Preferred") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR_PREF);
        } else if (strcasecmp(preset_str, "Solar") == 0) {
            thermos_new[idx].heat_src = static_cast<uint8_t>(network_heat_src_t::SOLAR);
        }
    }

    uint8_t const pool_heat_src = thermos_new[pool_idx].heat_src;
    uint8_t const spa_heat_src = thermos_new[spa_idx].heat_src;
    bool const thermos_changed = memcmp(thermos_old, thermos_new, sizeof(poolstate_thermo_t) * POOLSTATE_THERMO_TYP_COUNT) != 0;

        // actuate thermostat changes if necessary

    if (thermos_changed) {

        network_msg_t msg = {
            .typ = network_msg_typ_t::CTRL_HEAT_SET,
            .u = {
                .ctrl_heat_set = {
                    .poolSetpoint = thermos_new[pool_idx].set_point,
                    .spaSetpoint = thermos_new[spa_idx].set_point,
                    .heatSrc = static_cast<uint8_t>(pool_heat_src | (spa_heat_src << 2))
                },
            },
        };

        ESP_LOGV(TAG, "Sending HEAT_SET: pool_SP=%u°F, spa_SP=%u°F, heat_src=0x%02X", 
                  msg.u.ctrl_heat_set.poolSetpoint, 
                  msg.u.ctrl_heat_set.spaSetpoint,
                  msg.u.ctrl_heat_set.heatSrc);
        ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc());
    }
    
    // DON'T publish state here - wait for pool controller confirmation
    // State will be published by update_climates() when pool responds
}

void
OpnPoolClimate::add_pending_climate(bool has_setpoint, float setpoint_celsius,
                                  bool has_heat_src, uint8_t heat_src) {
    
        // add new pending (overwriting any existing pending for this climate)
    uint8_t idx = get_idx();
    ESP_LOGVV(TAG, "Adding pending climate: circuit=%u, has_setpoint=%d (%.1f°C), has_heat_src=%d (0x%02X)", 
              idx, has_setpoint, setpoint_celsius, has_heat_src, heat_src);
    
    pending_climate_ = {
        .is_pending = true,
        .target_setpoint_celsius = setpoint_celsius,
        .target_heat_src = heat_src,
        .has_setpoint_change = has_setpoint,
        .has_heat_src_change = has_heat_src,
        .timestamp = esp_timer_get_time()
    };
}

void
OpnPoolClimate::check_pending_climate(const poolstate_t * state) {
    
    pending_climate_t * const it = &pending_climate_;

    if (it->is_pending) {

        uint8_t idx = get_idx();
        poolstate_thermo_t const * const thermo = &state->thermos[idx];
        
        bool setpoint_matches = true;
        bool heat_src_matches = true;
        
            // check if setpoint was changed
        if (it->has_setpoint_change) {
            float actual_setpoint_fahrenheit = thermo->set_point;
            float target_setpoint_fahrenheit = it->target_setpoint_celsius * 9.0f / 5.0f + 32.0f;
            
            // Allow 1 degree tolerance due to rounding
            setpoint_matches = (abs(actual_setpoint_fahrenheit - target_setpoint_fahrenheit) <= 1.0f);
            
            ESP_LOGVV(TAG, "Climate setpoint check: circuit=%u, target=%.1f°F, actual=%.1f°F, matches=%d",
                        idx, target_setpoint_fahrenheit, actual_setpoint_fahrenheit, setpoint_matches);
        }
        
            // check if heat source was changed
        if (it->has_heat_src_change) {
            heat_src_matches = (thermo->heat_src == it->target_heat_src);
            
            ESP_LOGVV(TAG, "Climate heat_src check: circuit=%u, target=%u, actual=%u, matches=%d",
                        idx, it->target_heat_src, thermo->heat_src, heat_src_matches);
        }
        
            // check if both changes are confirmed
        if (setpoint_matches && heat_src_matches) {
            ESP_LOGVV(TAG, "Climate confirmed: circuit=%u", idx);
            
                // update and publish the climate state
            float water_temp_f = state->temps[static_cast<uint8_t>(poolstate_temp_typ_t::WATER)].temp;
            float water_temp_c = (water_temp_f - 32.0f) * 5.0f / 9.0f;
            
            this->current_temperature = water_temp_c;
            this->target_temperature = (thermo->set_point - 32.0f) * 5.0f / 9.0f;
            
                // update mode based on circuit state
            if (state->circuits.active[idx]) {
                if (thermo->heating) {
                    this->mode = climate::CLIMATE_MODE_HEAT;
                } else {
                    this->mode = climate::CLIMATE_MODE_AUTO;
                }
            } else {
                this->mode = climate::CLIMATE_MODE_OFF;
            }
            
                // update action
            if (thermo->heating) {
                this->action = climate::CLIMATE_ACTION_HEATING;
            } else if (state->circuits.active[idx]) {
                this->action = climate::CLIMATE_ACTION_IDLE;
            } else {
                this->action = climate::CLIMATE_ACTION_OFF;
            }
            
            it->is_pending = false;
        }
            // check for timeout (e.g., 15 seconds)
        else if (esp_timer_get_time() - it->timestamp > 15000000LL) {
            ESP_LOGW(TAG, "Climate confirmation timeout, circuit=%u", idx);
            
            // Publish current state anyway
            this->publish_state();
            it->is_pending = false;
        }
    }
}

}  // namespace opnpool
}  // namespace esphome