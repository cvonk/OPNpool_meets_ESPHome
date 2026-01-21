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

    void dump_config();
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
        
    poolstate_thermo_typ_t get_thermo_typ() const { return this->thermo_typ_; }

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
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome