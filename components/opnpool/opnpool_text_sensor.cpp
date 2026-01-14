#include <esphome/core/log.h>

#include "opnpool_text_sensor.h"
#include "network.h"
#include "ipc.h"

namespace esphome {
namespace opnpool {

static const char *TAG = "opnpool_text_sensor";

void OpnPoolTextSensor::setup() {
    // Nothing to do here - parent handles setup
}

void OpnPoolTextSensor::dump_config() {
    LOG_TEXT_SENSOR("  ", "Text Sensor", this);
}

void OpnPoolTextSensor::publish_state_if_changed(const std::string &state) {
    if (!last_state_valid_ || last_state_ != state) {
        this->publish_state(state);
        last_state_ = state;
        last_state_valid_ = true;
    }
}

}  // namespace opnpool
}  // namespace esphome