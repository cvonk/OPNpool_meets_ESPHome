#pragma once

#include <esphome/core/component.h>
#include <esphome/components/climate/climate.h>

namespace esphome {
namespace opnpool {

    // forward declarations
struct poolstate_t;
class OpnPool;  

class OpnPoolClimate : public climate::Climate, public Component {
    
  public:
    OpnPoolClimate(OpnPool* parent, uint8_t idx) : parent_{parent}, idx_{idx} {}
    void dump_config() override;
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
        
    uint8_t get_idx() const { return this->idx_; }
    void update_climate(const poolstate_t * state);
    void publish_value_if_changed(
        uint8_t const idx, float const value_current_temperature, 
        float const value_target_temperature, climate::ClimateMode const value_mode,
        char const * value_custom_preset, climate::ClimateAction const value_action
    );
    
  protected: 
    uint8_t const   idx_;
    OpnPool * const parent_;
    
    struct last_value_t {
        bool                    valid{false};
        float                   current_temp{0.0f};
        float                   target_temp{0.0f};
        char const *            custom_preset{nullptr};
        climate::ClimateMode    mode{climate::CLIMATE_MODE_OFF};
        climate::ClimateAction  action{climate::CLIMATE_ACTION_OFF};
    } last_value_;
};

}  // namespace opnpool
}  // namespace esphome