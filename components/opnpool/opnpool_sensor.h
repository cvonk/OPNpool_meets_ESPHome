#pragma once

#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolSensor : public sensor::Sensor, public Component {

  public:
    OpnPoolSensor(OpnPool* parent) : parent_{parent} {}
    
    void dump_config();
    
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    OpnPool * const  parent_;

    struct last_t {
        bool  valid;
        float value;
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome