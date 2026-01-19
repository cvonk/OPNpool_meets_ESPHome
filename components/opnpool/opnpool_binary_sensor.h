#pragma once

#include <esphome/core/component.h>
#include <esphome/components/binary_sensor/binary_sensor.h>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolBinarySensor : public binary_sensor::BinarySensor, public Component {

  public:
    OpnPoolBinarySensor(OpnPool* parent, uint8_t idx) : parent_{parent}, idx_{idx} {}
    void dump_config() override;
    
    void publish_value_if_changed(bool value);

  protected:
    uint8_t const   idx_;
    OpnPool * const parent_;

    struct last_t {
        bool valid{false};
        bool value{false};
    } last_;
};

}  // namespace opnpool
}  // namespace esphome