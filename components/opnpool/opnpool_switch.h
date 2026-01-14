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
    void setup() override;
    void dump_config() override;
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    void set_idx(uint8_t idx) { this->idx_ = idx; }
    uint8_t get_idx() const { return this->idx_; }
    
    void write_state(bool state) override;
    
    void publish_state_if_changed(bool state);
    void add_pending_switch(bool target_state);
    void check_pending_switch(const poolstate_t *new_state);
    void on_switch_command(bool const state);

  protected:
    uint8_t idx_{0};
    OpnPool * parent_{nullptr};

    bool last_state_{false};
    bool last_state_valid_{false};

        // management
    struct pending_switch_t {
        bool      is_pending;
        bool      target_state;
        int64_t   timestamp;
    } pending_switch_{};
};

}  // namespace opnpool
}  // namespace esphome