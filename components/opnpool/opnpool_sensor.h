#pragma once

#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolSensor : public sensor::Sensor, public Component {

  public:
    OpnPoolSensor(OpnPool* parent, uint8_t id) : parent_{parent}, sensor_id_{id} {}
    
    void dump_config();
    
    uint8_t get_sensor_id() const { return this->sensor_id_; }    
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    OpnPool * const parent_;
    uint8_t const   sensor_id_;

    struct last_t {
        bool  valid{false};
        float value{0.0f};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome