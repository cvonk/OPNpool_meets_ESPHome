/**
 * @file pool_task.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - pool_task: packetizes RS-485 byte stream from bus
 * 
 * @copyright Copyright (c) 2026 Coert Vonk
 * 
 * This file implements the FreeRTOS task logic for the OPNpool component, responsible for
 * managing RS-485 communication with the pool controller. It handles both receiving and
 * transmitting protocol packets, supporting two distinct message formats used by the controller:
 *   - A5 protocol: Framed messages with headers, length, data, and checksums.
 *   - IC protocol: Framed with 0x10 0x02 ... <data> ... <ch> 0x10 0x03.
 *
 * Core responsibilities include:
 * - Continuously reading from the RS-485 bus, packetizing incoming byte streams, and parsing
 *   them into higher-level datalink and network messages.
 * - Relaying received network messages to the main ESPHome task via IPC queues.
 * - Handling requests from the main task, converting them into protocol packets, and transmitting
 *   them to the pool controller.
 * - Managing a transmit queue for outgoing packets, ensuring correct half-duplex operation
 *   (using RTS/flow control) and echoing sent messages back up the protocol stack for state
 *   consistency.
 * - Periodically sending control and status requests (such as heat and schedule queries) to
 *   keep the pool state up to date.
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

#include <string.h>
#include <esp_system.h>
#include "esphome/core/log.h"
#include <esp_system.h>
#include <time.h>

#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "ipc.h"
#include "pool_task.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "pool_task";

static bool
_service_pkts_from_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    bool txOpportunity = false;
    datalink_pkt_t pkt;
    network_msg_t msg;

    if (datalink_rx_pkt(rs485, &pkt) == ESP_OK) {

        if (network_rx_msg(&pkt, &msg, &txOpportunity) == ESP_OK) {

            ipc_send_network_msg_to_main_task(&msg, ipc);
        }
        free(pkt.skb);
    }
    return txOpportunity;
}

static void
_service_requests_from_home(rs485_handle_t rs485, ipc_t const * const ipc)
{
    network_msg_t msg;

    if (xQueueReceive(ipc->to_pool_q, &msg, (TickType_t)0) == pdPASS) {

        datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));

        if (network_create_pkt(&msg, pkt) == ESP_OK) {

            datalink_tx_pkt_queue(rs485, pkt);  // pkt and pkt->skb freed by recipient
            return;
        }
        free(pkt);
    }
}

static void
_queue_req(rs485_handle_t const rs485, network_msg_typ_t const typ)
{
    network_msg_t msg = {
        .typ = typ
    };
    datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));

    if (network_create_pkt(&msg, pkt) == ESP_OK) {

        datalink_tx_pkt_queue(rs485, pkt);  // pkt and pkt->skb freed by mailbox recipient

    } else {
        free(pkt);
    }
}

static void
_forward_queued_pkt_to_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    datalink_pkt_t const * const pkt = rs485->dequeue(rs485);
    if (pkt) {
        ESP_LOGVV(TAG, "forward_queue: pkt typ=%s", datalink_typ_ctrl_str(static_cast<datalink_typ_ctrl_t>(pkt->typ)));

        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
              size_t const dbg_size = 128;
              char dbg[dbg_size];
              assert(pkt->skb);
              (void) skb_print(TAG, pkt->skb, dbg, dbg_size);
              ESP_LOGVV(TAG, "tx { %s}", dbg);
        }
        rs485->tx_mode(true);
        rs485->write_bytes(pkt->skb->priv.data, pkt->skb->len);
        rs485->tx_mode(false);

        // pretend that we received our own message

        bool txOpportunity = false;
        network_msg_t msg;

        ESP_LOGVV(TAG, "pretent rx: pkt typ=%s", datalink_typ_ctrl_str(static_cast<datalink_typ_ctrl_t>(pkt->typ)));

        if (network_rx_msg(pkt, &msg, &txOpportunity) == ESP_OK) {

            ipc_send_network_msg_to_main_task(&msg, ipc);
        }
        free(pkt->skb);
        free((void *) pkt);
    }
}

void
pool_req_task(void * rs485_void) 
{
    rs485_handle_t const rs485 = (rs485_handle_t)rs485_void;

    while (1) {
        _queue_req(rs485, network_msg_typ_t::CTRL_HEAT_REQ);
        _queue_req(rs485, network_msg_typ_t::CTRL_SCHED_REQ);
        vTaskDelay((TickType_t)30 * 1000 / portTICK_PERIOD_MS);
    }
}

void
pool_task(void * ipc_void)
{
    ESP_LOGI(TAG, "init ..");

    ipc_t * const ipc = static_cast<ipc_t*>(ipc_void);
    rs485_handle_t const rs485 = rs485_init(&ipc->config.rs485_pins);

    // request some initial information from the controller
    _queue_req(rs485, network_msg_typ_t::CTRL_VERSION_REQ);
    _queue_req(rs485, network_msg_typ_t::CTRL_TIME_REQ);

    // periodically request information from controller
    xTaskCreate(&pool_req_task, "pool_req_task", 2*4096, rs485, 5, NULL);

    ESP_LOGI(TAG, "started");

    while (1) {

        // read from ipc->to_pool_q

        _service_requests_from_home(rs485, ipc);

        // read from the rs485 device, until there is a packet,
        // then move the packet up the protocol stack to process it.

        if (_service_pkts_from_rs485(rs485, ipc)) {

            // there is a transmit opportunity after the pool controller
            // send a broadcast.  If there is rs485 transmit queue, then
            // create a network message and transmit it.

            _forward_queued_pkt_to_rs485(rs485, ipc);
        }
         vTaskDelay((TickType_t)100 / portTICK_PERIOD_MS);
    }
}

}  // namespace opnpool
}  // namespace esphome