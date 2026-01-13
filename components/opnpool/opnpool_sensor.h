#pragma once

#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolSensor : public sensor::Sensor, public Component {
  public:
    void setup() override;
    void dump_config() override;
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    
    void publish_state_if_changed(float state, float tolerance = 0.01f);

  protected:
    OpnPool *parent_{nullptr};
    float last_state_{0};
    bool last_state_valid_{false};
};

}  // namespace opnpool
}  // namespace esphome