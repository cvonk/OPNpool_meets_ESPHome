/**
 * @file opnpool_binary_sensor.cpp
 * @brief OPNpool - Reports digital pool sensors to Home Assistant.
 *
 * @details
 * Implements the binary sensor entity interface for the OPNpool component, allowing
 * ESPHome to monitor digital pool sensor states (such as pump status, error conditions,
 * etc.) and publish them to Home Assistant.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. The maximum number of binary sensors is limited
 * by the use of uint8_t for indexing.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esphome/core/log.h>

#include "opnpool_binary_sensor.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool.binary_sensor";

void OpnPoolBinarySensor::dump_config() {
    LOG_BINARY_SENSOR("  ", "Binary Sensor", this);
}

/**
 * @brief Publishes the binary sensor state to Home Assistant if it has changed.
 *
 * @details 
 * Compares the new binary sensor state with the last published state. If the state is not
 * yet valid, or if the new value differs from the last value, updates the internal state
 * and publishes the new value to Home Assistant. This avoids redundant updates to Home
 * Assistant.
 *
 * @param value The new binary sensor value to be published.
 */
void OpnPoolBinarySensor::publish_value_if_changed(bool const value)
{
    if (!last_value_.valid || last_value_.value != value) {

        ESP_LOGV(TAG, "Publishing binary sensor [%u] value: %s", idx_, value ? "ON" : "OFF");
        
        this->publish_state(value);

        last_value_ = {
            .valid = true,
            .value = value
        };
    }
}

}  // namespace opnpool
}  // namespace esphome