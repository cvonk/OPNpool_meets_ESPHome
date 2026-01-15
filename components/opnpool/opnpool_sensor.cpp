/**
 * @file opnpool_sensor.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Reports analog pool sensors to Home Assistant.
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
 * 
 * @details
 * Implements the sensor entity interface for the OPNpool component, allowing ESPHome
 * to monitor pool-related sensor values (such as temperatures, etc.) and publish them
 * to Home Assistant.
 * 
 * This file is part of OPNpool.
 * OPNpool is free software: you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 * OPNpool is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with OPNpool. 
 * If not, see <https://www.gnu.org/licenses/>.
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: Copyright 2026 Coert Vonk
 */

#include <esphome/core/log.h>
#include <cmath>

#include "opnpool_sensor.h"
#include "network.h"
#include "ipc.h"

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