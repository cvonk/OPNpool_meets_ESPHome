#pragma once

#include <esphome/core/component.h>
#include <esphome/components/binary_sensor/binary_sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolBinarySensor : public binary_sensor::BinarySensor, public Component {

  public:
    OpnPoolBinarySensor(OpnPool* parent, uint8_t id) : parent_{parent}, id_{static_cast<binary_sensor_id_t>(id)} {}
    void dump_config();
    
    binary_sensor_id_t get_binary_sensor_id() const { return this->id_; }    
    void publish_value_if_changed(bool value);

  protected:
    OpnPool * const          parent_;
    binary_sensor_id_t const id_;

    struct last_t {
        bool valid{false};
        bool value{false};
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome