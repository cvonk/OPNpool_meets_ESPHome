/**
 * @file ipc.cpp
 * @brief OPNpool - Inter Process Communication: mailboxes used to exchange messages between tasks
 * 
 * Thread safety is not provided, because it is not required for the single-threaded nature of ESPHome.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esphome/core/log.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "ipc.h"
#include "skb.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "ipc";

/**
 * @brief              Send a network message to the main task
 *
 * @param network_msg  Pointer to the network message to send
 * @param ipc          Pointer to the IPC structure containing the queue handles
 */

void
ipc_send_network_msg_to_main_task(network_msg_t const * const network_msg, ipc_t const * const ipc)
{
    if (xQueueSendToBack(ipc->to_main_q, network_msg, 0) != pdPASS) {
        ESP_LOGW(TAG, "to_main_q full");
    }
    vTaskDelay(1);  // give others a chance to catch up
}

/**
 * @brief              Send a network message to the pool task
 *
 * @param network_msg  Pointer to the network message to send
 * @param ipc          Pointer to the IPC structure containing the queue handles
 **/

void
ipc_send_network_msg_to_pool_task(network_msg_t const * const network_msg, ipc_t const * const ipc)
{
    if (xQueueSendToBack(ipc->to_pool_q, network_msg, 0) != pdPASS) {
        ESP_LOGW(TAG, "to_pool_q full");
    }
}

} // namespace opnpool
} // namespace esphome