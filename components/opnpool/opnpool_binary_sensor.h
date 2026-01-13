#pragma once

#include <esphome/core/component.h>
#include <esphome/components/binary_sensor/binary_sensor.h>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolBinarySensor : public binary_sensor::BinarySensor, public Component {
  public:
    void setup() override;
    void dump_config() override;
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    
    void publish_state_if_changed(bool state);

  protected:
    OpnPool *parent_{nullptr};
    bool last_state_{false};
    bool last_state_valid_{false};
};

}  // namespace opnpool
}  // namespace esphome