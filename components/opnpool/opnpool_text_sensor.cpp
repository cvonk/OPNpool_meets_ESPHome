/**
 * @file opnpool_sensor.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Reports digital pool text sensors to Home Assistant.
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
 * 
 * @details
 * This file implements the OPNpool text sensor integration for ESPHome, enabling the reporting of various
 * pool controller string values (such as schedules, firmware versions, and status messages) to Home Assistant.
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