/**
 * @file opnpool_binary_sensor.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Pool binary sensor interface
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
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
 * SPDX-FileCopyrightText: 2026 Coert Vonk
 */

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