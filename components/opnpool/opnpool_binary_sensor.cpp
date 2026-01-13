#include <esphome/core/log.h>

#include "opnpool_binary_sensor.h"
#include "network.h"
#include "ipc.h"

namespace esphome {
namespace opnpool {

static const char *TAG = "opnpool.binary_sensor";

void OpnPoolBinarySensor::setup() {
    // Nothing to do here - parent handles setup
}

void OpnPoolBinarySensor::dump_config() {
    LOG_BINARY_SENSOR("  ", "Binary Sensor", this);
}

void OpnPoolBinarySensor::publish_state_if_changed(bool state) {
    if (!last_state_valid_ || last_state_ != state) {
        this->publish_state(state);
        last_state_ = state;
        last_state_valid_ = true;
    }
}

}  // namespace opnpool
}  // namespace esphome