#pragma once

#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    OpnPoolTextSensor(OpnPool* parent) : parent_{parent} {}
    
    void dump_config();
    
    void publish_value_if_changed(const std::string &value);

  protected:
    OpnPool * const  parent_;

    struct last_t {
        bool         valid;
        std::string  value;
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome