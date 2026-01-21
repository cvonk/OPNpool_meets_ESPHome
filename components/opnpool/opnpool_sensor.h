#pragma once

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPoolSensor : public sensor::Sensor, public Component {

  public:
    OpnPoolSensor() {}
    
        // Called by ESPHome to dump the configuration of the component.
        // Set logger for this module to INFO or higher to see output.
    void dump_config();
    
        // called by the OpnPool component to update the sensor value
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    struct last_t {
        bool  valid;
        float value;
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome