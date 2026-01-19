#pragma once

#include <esphome/core/component.h>
#include <esphome/components/climate/climate.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

    // forward declarations
struct poolstate_t;
class OpnPool;  

class OpnPoolClimate : public climate::Climate, public Component {
    
  public:
    OpnPoolClimate(OpnPool* parent, uint8_t id) : parent_{parent}, climate_id_{id} {}

    void dump_config();
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
        
    uint8_t get_climate_id() const { return this->climate_id_; }
    void publish_value_if_changed(
        uint8_t const climate_id, float const value_current_temperature, 
        float const value_target_temperature, climate::ClimateMode const value_mode,
        char const * value_custom_preset, climate::ClimateAction const value_action
    );
    
  protected: 
    OpnPool * const parent_;
    uint8_t const   climate_id_;
    
    struct last_t {
        bool                    valid{false};
        float                   current_temp{0.0f};
        float                   target_temp{0.0f};
        char const *            custom_preset{nullptr};
        climate::ClimateMode    mode{climate::CLIMATE_MODE_OFF};
        climate::ClimateAction  action{climate::CLIMATE_ACTION_OFF};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome