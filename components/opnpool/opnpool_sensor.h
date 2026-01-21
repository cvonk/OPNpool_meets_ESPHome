#pragma once

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

    // forward declaration (to avoid circular dependencies)
class OpnPool;

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