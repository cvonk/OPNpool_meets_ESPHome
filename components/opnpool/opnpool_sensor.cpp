#include <esphome/core/log.h>
#include <cmath>

#include "opnpool_sensor.h"
#include "network.h"
#include "ipc.h"

namespace esphome {
namespace opnpool {

static const char *TAG = "opnpool.sensor";

void OpnPoolSensor::setup() {
    // Nothing to do here - parent handles setup
}

void OpnPoolSensor::dump_config() {
    LOG_SENSOR("  ", "Sensor", this);
}

void OpnPoolSensor::publish_state_if_changed(float state, float tolerance) {
    if (!last_state_valid_ || fabs(last_state_ - state) > tolerance) {
        this->publish_state(state);
        last_state_ = state;
        last_state_valid_ = true;
    }
}

}  // namespace opnpool
}  // namespace esphome