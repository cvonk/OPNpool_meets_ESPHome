/**
 * @file opnpool_switch.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Pool switch interface
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
 * SPDX-FileCopyrightText: Copyright 2026 Coert Vonk
 */

#include <esp_timer.h>
#include <esphome/core/log.h>

#include "opnpool_switch.h"  // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpool_state.h"

namespace esphome {
namespace opnpool {

static const char *TAG = "opnpool_switch";

void OpnPoolSwitch::setup() {
    // Nothing to do here - parent takes care of me
}

void OpnPoolSwitch::dump_config() {
    LOG_SWITCH("  ", "Switch", this);
}

void OpnPoolSwitch::publish_state_if_changed(bool state) {
    if (!last_state_valid_ || last_state_ != state) {
        this->publish_state(state);
        last_state_ = state;
        last_state_valid_ = true;
    }
}

void OpnPoolSwitch::on_switch_command(bool const state) {

    network_msg_t msg = {
        .typ = network_msg_typ_t::CTRL_CIRCUIT_SET,
        .u = {
            .ctrl_circuit_set = {
              .circuit = static_cast<uint8_t>(this->get_idx() + 1),
              .value = state ? (uint8_t)1 : (uint8_t)0,          
            },
        },
    };

    ESP_LOGVV(TAG, "Sending CIRCUIT_SET command: circuit=%u to %u", msg.u.ctrl_circuit_set.circuit, msg.u.ctrl_circuit_set.value);
    ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc());
}

void OpnPoolSwitch::write_state(bool state) {

  if (this->parent_) {
    
        // send command but DON'T publish yet
    this->on_switch_command(state);
    
       // store as pending
    this->add_pending_switch(state);
  }

}

void
OpnPoolSwitch::add_pending_switch(bool const target_state) {

        // add new pending (overwriting any existing pending for this switch)
    uint8_t idx = this->get_idx();
    ESP_LOGVV(TAG, "Adding pending switch: circuit=%u, target_state=%d", idx, target_state);
    
    pending_switch_ = {
        .is_pending = true,
        .target_state = target_state,
        .timestamp = esp_timer_get_time()
    };
}

void
OpnPoolSwitch::check_pending_switch(poolstate_t const * const state) {

    pending_switch_t * const it = &this->pending_switch_;

    if (it->is_pending) {    
        uint8_t idx = get_idx();
        bool current_state = state->circuits.active[idx];
        if (current_state == it->target_state) {
            ESP_LOGVV(TAG, "Pending switch applied: circuit=%u, state=%d", idx, current_state);
            it->is_pending = false;
            this->publish_state(current_state);
        }
    }
}


}  // namespace opnpool
}  // namespace esphome