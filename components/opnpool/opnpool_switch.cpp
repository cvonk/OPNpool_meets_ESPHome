/**
 * @file opnpool_switch.cpp
 * @brief OPNpool - Actuates switch settings from Home Assistant on the pool controller.
 * 
 * @details
 * This file implements the switch entity interface for the OPNpool component, enabling
 * control and monitoring of pool circuits (such as POOL, SPA, AUX1).
 * It provides methods to handle switch state changes initiated by Home Assistant,
 * constructs and sends protocol messages to the pool controller over RS-485, and
 * updates the switch state based on controller feedback.
 * The implementation ensures that only meaningful state changes are published to
 * Home Assistant, avoiding redundant updates.
 * 
 * Thread safety is not provided, because it is not required for the single-threaded nature of ESPHome.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_timer.h>
#include <esphome/core/log.h>

#include "opnpool_switch.h"   // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpool_state.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_switch";

void OpnPoolSwitch::setup() {
    // nothing to do here - my parent takes care of me :-)
}

void OpnPoolSwitch::dump_config()
{
    LOG_SWITCH("  ", "Switch", this);
}

/**
 * @brief
 * Handles switch state changes triggered by Home Assistant.
 *
 * @details
 * Constructs and sends a network message to the pool controller to set the state of the
 * specified circuit. The circuit index is mapped to the pool controller's circuit numbering.
 * The message is sent via IPC to the pool_task for RS-485 transmission. This method is
 * called automatically when the switch entity is toggled in Home Assistant or ESPHome.
 *
 * @param state The desired state of the switch (true for ON, false for OFF).
 */
void OpnPoolSwitch::write_state(bool state)
{
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

/**
 * @brief
 * Updates the switch state from the latest pool controller feedback.
 *
 * @details
 * Retrieves the current state of the specified circuit from the pool controller's
 * state structure.
 * Compares the received state with the last published state and publishes the new
 * value to Home Assistant only if it has changed.
 *
 * @param state Pointer to the latest poolstate_t structure containing updated pool controller data.
 */
void
OpnPoolSwitch::update_switch(poolstate_t const * const state)
{
    uint8_t const idx = get_idx();
    bool current_state = state->circuits.active[idx];

    publish_value_if_changed(current_state);
}

/**
 * @brief
 * Publishes the switch state to Home Assistant if it has changed.
 *
 * @details
 * Compares the new switch state with the last published state. If the state has changed or
 * is not yet valid, updates the internal state and publishes the new value to Home Assistant.
 * This avoids redundant updates to Home Assistant.
 *
 * @param value The new state of the switch (true for ON, false for OFF).
 */
void OpnPoolSwitch::publish_value_if_changed(bool value)
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