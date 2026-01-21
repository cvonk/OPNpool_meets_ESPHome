/**
 * @file opnpool_switch.cpp
 * @brief Actuates switch settings from Home Assistant on the pool controller.
 *
 * @details
 * This file implements the switch entity interface for the OPNpool component, enabling
 * control and monitoring of pool circuits (such as POOL, SPA, AUX1). It provides methods
 * to handle switch state changes initiated by Home Assistant, constructs and sends
 * protocol messages to the pool controller over RS-485, and updates the switch state
 * based on controller feedback. The implementation ensures that only meaningful state
 * changes are published to Home Assistant, avoiding redundant updates.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. The maximum number of switches is limited
 * by the use of uint8_t for indexing.
 *
 * restore_mode is ignored because state is always synchronized from the pool controller.
 * The component will always reflect the actual state of the pool circuits as reported by
 * the controller.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esphome/core/log.h>

#include "opnpool_switch.h"   // no other #includes that could make a circular dependency
#include "opnpool.h"          // no other #includes that could make a circular dependency
#include "ipc.h"              // no other #includes that could make a circular dependency
#include "network_msg.h"      // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "opnpool_ids.h"      // conversion helper
#include "enum_helpers.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

static char const * const TAG = "opnpool_switch";

/**
 * @brief Dump the configuration and last known state of the switch entity.
 *
 * @details
 * Logs the configuration details for this switch, including its mapped circuit, ID,
 * and last known state (ON/OFF or Unknown). This information is useful for diagnostics
 * and debugging, providing visibility into the entity's state and configuration at
 * runtime.
 */

void
OpnPoolSwitch::dump_config()
{
    network_pool_circuit_t circuit = circuit_;

    LOG_SWITCH("  ", "Switch", this);
    ESP_LOGCONFIG(TAG, "    Circuit: %s", enum_str(circuit));
    ESP_LOGCONFIG(TAG, "    Last state: %s", last_.valid ? (last_.value ? "ON" : "OFF") : "Unknown");
}

/**
 * @brief
 * Handles switch state changes triggered by Home Assistant.
 *
 * @details
 * Constructs and sends a network message to the pool controller to set the state of the
 * specified circuit. The circuit index is mapped to the pool controller's circuit
 * numbering. The message is sent via IPC to the pool_task for RS-485 transmission. This
 * method is called automatically when the switch entity is toggled in Home Assistant or
 * ESPHome.
 *
 * @param state The desired state of the switch (true for ON, false for OFF).
 */
void
OpnPoolSwitch::write_state(bool value)
{
    network_pool_circuit_t const circuit = circuit_;
    uint8_t const circuit_idx = enum_index(circuit);

    network_msg_t msg = {
        .typ = network_msg_typ_t::CTRL_CIRCUIT_SET,
        .u = {
            .ctrl_circuit_set = {
                .circuit_plus_1 = static_cast<uint8_t>(circuit_idx + uint8_t(1)),
                .value = static_cast<uint8_t>(value ? 1 : 0),          
            },
        },
    };

    ESP_LOGVV(TAG, "Sending CIRCUIT_SET command: circuit+1=%u to %u", msg.u.ctrl_circuit_set.circuit_plus_1, msg.u.ctrl_circuit_set.value);
    ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc());    

    // DON'T publish state here - wait for pool controller confirmation
}

/**
 * @brief
 * Publishes the switch state to Home Assistant if it has changed.
 *
 * @details
 * Compares the new switch state with the last published state. If the state has changed
 * or is not yet valid, updates the internal state and publishes the new value to Home
 * Assistant. This avoids redundant updates to Home Assistant.
 *
 * @param value The new state of the switch (true for ON, false for OFF).
 */
void
OpnPoolSwitch::publish_value_if_changed(bool value)
{
    if (!last_.valid || last_.value != value) {

        this->publish_state(value);

        last_ = {
            .valid = true,
            .value = value
        };
        
        network_pool_circuit_t const circuit = circuit_;
        ESP_LOGV(TAG, "Published %s: %s", enum_str(circuit), value ? "ON" : "OFF");
    }
}

}  // namespace opnpool
}  // namespace esphome