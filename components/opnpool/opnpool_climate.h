#pragma once

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>
#include <esphome/components/climate/climate.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct poolstate_t;
class OpnPool;  

class OpnPoolClimate : public climate::Climate, public Component {
    
  public:
    OpnPoolClimate(OpnPool* parent, uint8_t id) : parent_{parent}, id_{static_cast<climate_id_t>(id)} {}

        // Called by ESPHome to dump the configuration of the component.
        // Set logger for this module to INFO or higher to see output.
    void dump_config();

        // called by ESPHome to get the traits of the climate component
    climate::ClimateTraits traits() override;

        // called by ESPHome to control the climate component
    void control(const climate::ClimateCall &call) override;
        
        // called by the OpnPool component to get the thermostat index
    poolstate_thermo_typ_t get_thermo_typ() const { return this->thermo_typ_; }

        // called by the OpnPool component to update the climate state
    void publish_value_if_changed(
        float const value_current_temperature, 
        float const value_target_temperature, climate::ClimateMode const value_mode,
        char const * value_custom_preset, climate::ClimateAction const value_action
    );

  protected: 
    OpnPool * const              parent_;
    climate_id_t const           id_;
    poolstate_thermo_typ_t const thermo_typ_ = climate_id_to_poolstate_thermo(id_);
    
    struct last_t {
        bool                    valid;
        float                   current_temp;
        float                   target_temp;
        char const *            custom_preset;
        climate::ClimateMode    mode;
        climate::ClimateAction  action;
    } last_ = {
        .valid = false,
        .current_temp = 0.0f,
        .target_temp = 0.0f,
        .custom_preset = nullptr,
        .mode = climate::CLIMATE_MODE_OFF,
        .action = climate::CLIMATE_ACTION_OFF
    };
};

}  // namespace opnpool
}  // namespace esphome