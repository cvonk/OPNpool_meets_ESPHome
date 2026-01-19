#pragma once

#include <esphome/core/component.h>
#include <esphome/components/binary_sensor/binary_sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolBinarySensor : public binary_sensor::BinarySensor, public Component {

  public:
    OpnPoolBinarySensor(OpnPool* parent) : parent_{parent} {}
    void dump_config();
    
    void publish_value_if_changed(bool value);

  protected:
    OpnPool * const  parent_;

    struct last_t {
        bool valid;
        bool value;
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome