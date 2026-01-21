#pragma once

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    OpnPoolTextSensor() {}
    
        // Called by ESPHome to dump the configuration of the component.
        // Set logger for this module to INFO or higher to see output.
    void dump_config();
    
        // called by the OpnPool component to update the sensor value
    void publish_value_if_changed(const std::string &value);

  protected:
    struct last_t {
        bool         valid;
        std::string  value;
    } last_ = {
        .valid = false,
        .value = ""
    };
};

}  // namespace opnpool
}  // namespace esphome