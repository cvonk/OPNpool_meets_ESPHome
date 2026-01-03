#include "opnpool.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opnpool {

static const char *const TAG = "opnpool";

void OpnPool::setup() {
  for (auto *pin : switch_pins_) {
    pin->setup();
    pin->digital_write(false);
  }
  for (auto &s : sensors_) {
    s.pin->setup();
  }
}

void OpnPool::loop() {
  for (auto &s : sensors_) {
    bool current_state = s.pin->digital_read();
    if (current_state != s.sensor->state) {
      s.sensor->publish_state(current_state);
    }
  }
}

}  // namespace opnpool
}  // namespace esphome