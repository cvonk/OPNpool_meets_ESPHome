#pragma once

#include <esp_system.h>
#include <esp_types.h>
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
    OpnPoolSwitch(OpnPool* parent, uint8_t id) : parent_{parent}, id_{static_cast<switch_id_t>(id)} {}

        // Called by ESPHome to dump the configuration of the component.
        // Set logger for this module to INFO or higher to see output.
    void dump_config();
    
        // called by ESPHome to change the state of the switch
    void write_state(bool state) override;
    
        // called by the OpnPool component to update the switch state
    void publish_value_if_changed(bool const new_value);

  protected:
    OpnPool * const              parent_;
    switch_id_t const            id_;
    network_pool_circuit_t const circuit_ = switch_id_to_network_circuit(id_);

    struct last_t {
        bool valid;
        bool value;
    } last_ = {
        .valid = false,
        .value = false
    };
};

}  // namespace opnpool
}  // namespace esphome
