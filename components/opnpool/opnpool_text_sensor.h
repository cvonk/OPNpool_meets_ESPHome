#pragma once

#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    void setup() override;
    void dump_config() override;
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    void publish_value_if_changed(const std::string &value);


  protected:
    OpnPool *  parent_{nullptr};

    struct last_value_t {
        bool         valid{false};
        std::string  value{""};
    } last_value_;
};

}  // namespace opnpool
}  // namespace esphome