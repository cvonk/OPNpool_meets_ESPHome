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
    
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    OpnPool * parent_{nullptr};

    struct last_value_t {
        bool  valid{false};
        float value{false};
    } last_value_;
};

}  // namespace opnpool
}  // namespace esphome