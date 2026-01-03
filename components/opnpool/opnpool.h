#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
// These require the DEPENDENCIES in __init__.py to work
#include "esphome/components/switch/switch.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>

namespace esphome {
namespace opnpool {

class OpnPoolSwitch : public switch_::Switch {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
 protected:
  void write_state(bool state) override {
    pin_->digital_write(state);
    publish_state(state);
  }
  InternalGPIOPin *pin_;
};

struct BinarySensorData {
  binary_sensor::BinarySensor *sensor;
  InternalGPIOPin *pin;
};

class OpnPool : public Component {
 public:
  void add_switch(OpnPoolSwitch *sw, InternalGPIOPin *pin) {
    sw->set_pin(pin);
    switches_.push_back(sw);
    switch_pins_.push_back(pin);
  }
  
  void add_binary_sensor(binary_sensor::BinarySensor *sens, InternalGPIOPin *pin) {
    sensors_.push_back({sens, pin});
  }

  void setup() override;
  void loop() override;

 protected:
  std::vector<OpnPoolSwitch *> switches_;
  std::vector<InternalGPIOPin *> switch_pins_;
  std::vector<BinarySensorData> sensors_;
};

}  // namespace opnpool
}  // namespace esphome