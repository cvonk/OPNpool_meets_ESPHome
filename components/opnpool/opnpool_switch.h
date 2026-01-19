#pragma once

#include <esphome/core/component.h>
#include <esphome/components/switch/switch.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

    // forward declarations
struct poolstate_t;
class OpnPool;  

class OpnPoolSwitch : public switch_::Switch, public Component {
  public:
    OpnPoolSwitch(OpnPool* parent, uint8_t id) : parent_{parent}, switch_id_{id} {}

    void dump_config();
    void write_state(bool state) override;  // change triggered by Home Assistant
    
    uint8_t get_switch_id() const { return this->switch_id_; }    
    void publish_value_if_changed(bool const new_value);

  protected:
    OpnPool * const parent_;
    uint8_t const switch_id_;

    struct last_t {
        bool valid{false};
        bool value{false};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome
