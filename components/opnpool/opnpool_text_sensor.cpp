/**
 * @file opnpool_sensor.cpp
 * @brief OPNpool - Reports digital pool text sensors to Home Assistant.
 * 
 * @details
 * This file implements the OPNpool text sensor integration for ESPHome, enabling the reporting of various
 * pool controller string values (such as schedules, firmware versions, and status messages) to Home Assistant.
 * 
 * Thread safety is not provided, because it is not required for the single-threaded nature of ESPHome.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esphome/core/log.h>

#include "opnpool_text_sensor.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_text_sensor";

void OpnPoolTextSensor::setup()
{
    // nothing to do here - parent handles setup
}

void OpnPoolTextSensor::dump_config()
{
    LOG_TEXT_SENSOR("  ", "Text Sensor", this);
}

/**
 * @brief
 * Publishes the text sensor state to Home Assistant if it has changed.
 *
 * @details
 * Compares the new text sensor value with the last published value. If the state is not yet valid,
 * or if the new value differs from the last value, updates the internal state and publishes the new
 * value to Home Assistant. This avoids redundant updates to Home Assistant.
 *
 * @param value The new text sensor value to be published.
 */
void OpnPoolTextSensor::publish_value_if_changed(const std::string & value)
{
    if (!last_value_.valid || last_value_.value != value) {
        
        this->publish_state(value);
        last_value_ = {
            .valid = true,
            .value = value
        };
    }
}

}  // namespace opnpool
}  // namespace esphome