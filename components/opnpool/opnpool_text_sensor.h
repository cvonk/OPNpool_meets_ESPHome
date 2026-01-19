#pragma once

#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    OpnPoolTextSensor(OpnPool* parent, uint8_t idx) : parent_{parent}, idx_{idx} {}
    
    void dump_config() override;
    
    void publish_value_if_changed(const std::string &value);

  protected:
    uint8_t const   idx_;
    OpnPool * const parent_;

    struct last_t {
        bool         valid{false};
        std::string  value{""};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome