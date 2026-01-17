#pragma once

#include <esphome/core/component.h>
#include <esphome/components/switch/switch.h>

namespace esphome {
namespace opnpool {

    // forward declarations
struct poolstate_t;
class OpnPool;  

class OpnPoolSwitch : public switch_::Switch, public Component {
  public:
    OpnPoolSwitch(OpnPool* parent, uint8_t idx) : parent_{parent}, idx_{idx} {}

    void dump_config() override;
    void write_state(bool state) override;  // change triggered by Home Assistant
    
    uint8_t get_idx() const { return this->idx_; }    
    void update_switch(const poolstate_t *new_state);
    void publish_value_if_changed(bool const new_value);

  protected:
    uint8_t const idx_;
    OpnPool * const parent_;

    struct last_value_t {
        bool valid{false};
        bool value{false};
    } last_value_;
};

}  // namespace opnpool
}  // namespace esphome
