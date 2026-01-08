/**
 * @brief OPNpool, packet_task: packetizes RS-485 byte stream from bus
 *
 * The Pool controller uses two different protocols to communicate with its peripherals:
 *   - 	A5 has messages such as 0x00 0xFF <ldb> <sub> <dst> <src> <cfi> <len> [<data>] <chH> <ckL>
 *   -  IC has messages such as 0x10 0x02 <data0> <data1> <data2> .. <dataN> <ch> 0x10 0x03
 *
 * © Copyright 2014, 2019, 2022, Coert Vonk
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
 * SPDX-FileCopyrightText: Copyright 2014,2019,2022 Coert Vonk
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

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

static char const * const TAG = "pool_task";

static bool
_service_pkts_from_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    bool txOpportunity = false;
    datalink_pkt_t pkt;
    network_msg_t msg;

    if (datalink_rx_pkt(rs485, &pkt) == ESP_OK) {

        if (network_rx_msg(&pkt, &msg, &txOpportunity) == ESP_OK) {

// 2BD: it makes more sense to the msg in as data instead of a pointer
// then it gets copied into the queue instead of just passing a pointer

            ipc_send_network_msg_to_home(&msg, ipc);
        }
        free(pkt.skb);
    }
    return txOpportunity;
}

static void
_service_requests_from_home(rs485_handle_t rs485, ipc_t const * const ipc)
{
    ipc_to_pool_msg_t queued_msg;

    if (xQueueReceive(ipc->to_pool_q, &queued_msg, (TickType_t)0) == pdPASS) {

        switch(queued_msg.typ) {
            case IPC_TO_POOL_TYP_NETWORK_MSG: {
                network_msg_t * const msg = &queued_msg.u.network_msg;

                ESP_LOGV(TAG, "Handling msg typ=%u", ipc_to_pool_typ_str(msg->typ));

                // 2BD: handle this received network message

                // along the lines of the old `hass_create_message()`

                break;
            }
            default:
                ESP_LOGW(TAG, "Unknown msg typ: %u", queued_msg.typ);
                break;
        }
    }
}

static void
_queue_req(rs485_handle_t const rs485, network_msg_typ_t const typ)
{
    network_msg_t msg = {
        .typ = typ
    };
    datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));

    if (network_create_msg(&msg, pkt)) {
        datalink_tx_pkt_queue(rs485, pkt);  // pkt and pkt->skb freed by mailbox recipient
    } else {
        ESP_LOGE(TAG, "%s network_tx_typ failed", __func__);
        free(pkt);
    }
}

static void
_forward_queued_pkt_to_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    datalink_pkt_t const * const pkt = rs485->dequeue(rs485);
    if (pkt) {
        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
            size_t const dbg_size = 128;
            char dbg[dbg_size];
            assert(pkt->skb);
            (void) skb_print(TAG, pkt->skb, dbg, dbg_size);
            ESP_LOGV(TAG, "tx { %s}", dbg);
        }
        rs485->tx_mode(true);
        rs485->write_bytes(pkt->skb->priv.data, pkt->skb->len);
        rs485->tx_mode(false);

        // pretent that we received our own message

        bool txOpportunity = false;
        network_msg_t msg;

        if (network_rx_msg(pkt, &msg, &txOpportunity) == ESP_OK) {

            ipc_send_network_msg_to_home(&msg, ipc);

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
        _queue_req(rs485, MSG_TYP_CTRL_HEAT_REQ);
        _queue_req(rs485, MSG_TYP_CTRL_SCHED_REQ);
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
    _queue_req(rs485, MSG_TYP_CTRL_VERSION_REQ);
    _queue_req(rs485, MSG_TYP_CTRL_TIME_REQ);

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
        vTaskDelay(1);
    }
}

}  // namespace opnpool
}  // namespace esphome