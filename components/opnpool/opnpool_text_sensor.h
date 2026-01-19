#pragma once

#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    OpnPoolTextSensor(OpnPool* parent, uint8_t id) : parent_{parent}, text_sensor_id_{id} {}
    
    void dump_config();
    
    uint8_t get_text_sensor_id() const { return this->text_sensor_id_; }    
    void publish_value_if_changed(const std::string &value);

  protected:
    OpnPool * const parent_;
    uint8_t const   text_sensor_id_;

    struct last_t {
        bool         valid{false};
        std::string  value{""};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome