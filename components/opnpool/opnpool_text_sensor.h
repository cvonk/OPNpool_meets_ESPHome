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
    OpnPoolTextSensor(OpnPool* parent, uint8_t id) : parent_{parent}, id_{static_cast<text_sensor_id_t>(id)} {}
    
    void dump_config();
    
    text_sensor_id_t get_text_sensor_id() const { return this->id_; }    
    void publish_value_if_changed(const std::string &value);

  protected:
    OpnPool * const        parent_;
    text_sensor_id_t const id_;

    struct last_t {
        bool         valid{false};
        std::string  value{""};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome