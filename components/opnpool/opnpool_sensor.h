#pragma once

#include <esphome/core/component.h>
#include <esphome/components/sensor/sensor.h>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolSensor : public sensor::Sensor, public Component {

  public:
    OpnPoolSensor(OpnPool* parent, uint8_t idx) : parent_{parent}, idx_{idx} {}
    
    void dump_config() override;
    
    void publish_value_if_changed(float value, float tolerance = 0.01f);

  protected:
    uint8_t const   idx_;
    OpnPool * const parent_;

    struct last_t {
        bool  valid{false};
        float value{0.0f};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome