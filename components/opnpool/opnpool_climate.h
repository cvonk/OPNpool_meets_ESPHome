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
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    void set_idx(uint8_t idx) { this->idx_ = idx; }
    uint8_t get_idx() const { return this->idx_; }
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
    
    void publish_state_if_changed();
    void add_pending_climate(bool has_setpoint, float setpoint_celsius, bool has_heat_src, uint8_t heat_src);
    void check_pending_climate(const poolstate_t *new_state);

  protected:
    uint8_t idx_{0};
    OpnPool *parent_{nullptr};
    
    float last_current_temp_{0};
    float last_target_temp_{0};
    climate::ClimateMode last_mode_{climate::CLIMATE_MODE_OFF};
    climate::ClimateAction last_action_{climate::CLIMATE_ACTION_OFF};
    bool last_state_valid_{false};

        // management
    struct pending_climate_t {
        bool     is_pending;
        float    target_setpoint_celsius;
        uint8_t  target_heat_src;
        bool     has_setpoint_change;
        bool     has_heat_src_change;
        int64_t  timestamp;
    } pending_climate_{};
};

}  // namespace opnpool
}  // namespace esphome