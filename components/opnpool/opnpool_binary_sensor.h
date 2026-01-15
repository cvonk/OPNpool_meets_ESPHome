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
    
    void publish_value_if_changed(bool value);

  protected:
    OpnPool * parent_{nullptr};

    struct last_value_t {
        bool valid{false};
        bool value{false};
    } last_value_;
};

}  // namespace opnpool
}  // namespace esphome