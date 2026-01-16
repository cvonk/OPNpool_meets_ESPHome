/**
 * @file opnpool_sensor.cpp
 * @brief OPNpool - Reports analog pool sensors to Home Assistant.
 * 
 * @details
 * Implements the sensor entity interface for the OPNpool component, allowing ESPHome
 * to monitor pool-related sensor values (such as temperatures, etc.) and publish them
 * to Home Assistant.
 * 
 * Thread safety is not provided, because it is not required for the single-threaded nature of ESPHome.
 * 
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esphome/core/log.h>

#include "opnpool_sensor.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_sensor";

void OpnPoolSensor::setup()
{
    // nothing to do here - my parent takes care of me :-)
}

void OpnPoolSensor::dump_config()
{
    LOG_SENSOR("  ", "Sensor", this);
}

/**
 * @brief Publishes the sensor state to Home Assistant if it has changed.
 *
 * @details
 * Compares the new sensor value with the last published value. If the state is
 * not yet valid, or if the absolute difference between the new value and the last
 * value exceeds the given tolerance, updates the internal state and publishes the
 * new value to Home Assistant. This avoids redundant updates to Home Assistant.
 *
 * @param value      The new sensor value to be published.
 * @param tolerance  The minimum change required to trigger a new state publication.
 */
void OpnPoolSensor::publish_value_if_changed(float value, float tolerance)
{
    if (!last_value_.valid || fabs(last_value_.value - value) > tolerance) {

        this->publish_state(value);
        
        last_value_ = {
            .valid = true,
            .value = value
        };
    }
}

}  // namespace opnpool
}  // namespace esphome