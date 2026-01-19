#pragma once

#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolSensor : public sensor::Sensor, public Component {

  public:
    OpnPoolSensor(OpnPool* parent, uint8_t id) : parent_{parent}, id_{static_cast<sensor_id_t>(id)} {}
    
    void dump_config();
    
    sensor_id_t get_sensor_id() const { return this->id_; }    
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    OpnPool * const   parent_;
    sensor_id_t const id_;

    struct last_t {
        bool  valid;
        float value;
    } last_ = {
        .valid = false
    };
};

}  // namespace opnpool
}  // namespace esphome