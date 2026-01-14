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
    void setup() override;
    void dump_config() override;
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
        
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    void set_idx(uint8_t idx) { this->idx_ = idx; }
    uint8_t get_idx() const { return this->idx_; }

    void update_climate(const poolstate_t * state);
    void publish_state_if_changed(uint8_t const idx, float const new_current_temperature, float const new_target_temperature, climate::ClimateMode const new_mode, char const * new_custom_preset, climate::ClimateAction const new_action);
    
  protected:
    uint8_t idx_{0};
    OpnPool *parent_{nullptr};
    
    struct last_state_t {
        bool valid{false};
        float current_temp{0.0f};
        float target_temp{0.0f};
        climate::ClimateMode mode{climate::CLIMATE_MODE_OFF};
        char const * custom_preset{nullptr};
        climate::ClimateAction action{climate::CLIMATE_ACTION_OFF};
    } last_state_;
};

}  // namespace opnpool
}  // namespace esphome