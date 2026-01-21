#pragma once

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>
#include <esphome/components/binary_sensor/binary_sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPoolBinarySensor : public binary_sensor::BinarySensor, public Component {

  public:
    OpnPoolBinarySensor() {}

        // Called by ESPHome to dump the configuration of the component.
        // Set logger for this module to INFO or higher to see output.
    void dump_config();
    
        // called by the OpnPool component to update the sensor value
    void publish_value_if_changed(bool value);

  protected:
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