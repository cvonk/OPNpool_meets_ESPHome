/**
 * @file pool_task.cpp
 * @brief pool_task: packetizes RS-485 byte stream from bus
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
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. 
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include "esphome/core/log.h"
#include <string.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "network_msg.h"
#include "ipc.h"
#include "pool_task.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

static char const * const TAG = "pool_task";

/**
 * @brief Processes incoming packets from the RS-485 bus and relays messages to the main
 * task.
 *
 * Receives a packet from RS-485, decodes it into a network message, and sends it to the
 * main task via IPC if successful. Frees the packet buffer after processing.
 *
 * @param rs485 RS-485 handle.
 * @param ipc   IPC structure pointer.
 * @return true if a transmit opportunity is available, false otherwise.
 */
[[nodiscard]] static bool
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

/**
 * @brief Handles requests from the main task and queues them for RS-485 transmission.
 *
 * Receives network messages from the IPC queue, packetizes them, and queues them for
 * transmission to the pool controller. Frees the packet if creation fails.
 *
 * @param rs485 RS-485 handle.
 * @param ipc   IPC structure pointer.
 */
static void
_service_requests_from_main(rs485_handle_t rs485, ipc_t const * const ipc)
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

/**
 * @brief Queues a network message for transmission to the pool controller.
 *
 * Creates a network message of the specified type, packetizes it, and queues it for
 * transmission on the RS-485 bus. Frees the packet if creation fails.
 *
 * @param rs485 RS-485 handle.
 * @param typ   Network message type to send.
 */
static void
_queue_req(rs485_handle_t const rs485, network_msg_typ_t const typ)
{
    network_msg_t msg = {};  // prevents -Wmissing-field-initializers
    msg.typ = typ;

    datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));

    if (network_create_pkt(&msg, pkt) == ESP_OK) {

        datalink_tx_pkt_queue(rs485, pkt);  // pkt and pkt->skb freed by mailbox recipient

    } else {
        free(pkt);
    }
}

/**
 * @brief Forwards a queued packet from the transmit queue to the RS-485 bus.
 *
 * Dequeues a packet, transmits it over RS-485, logs the transmission if verbose,
 * and simulates reception for protocol state consistency. Frees the packet after use.
 *
 * @param rs485 RS-485 handle.
 * @param ipc   IPC structure pointer.
 */
static void
_forward_queued_pkt_to_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    datalink_pkt_t const * const pkt = rs485->dequeue(rs485);
    if (pkt) {
        ESP_LOGVV(TAG, "forward_queue: pkt typ=%s", enum_str(static_cast<datalink_typ_ctrl_t>(pkt->typ)));

        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
            size_t const dbg_size = 128;
            char dbg[dbg_size];
            if (pkt->skb == nullptr) {
                ESP_LOGE(TAG, "Packet skb is null");
            } else {
                (void) skb_print(TAG, pkt->skb, dbg, dbg_size);
                ESP_LOGVV(TAG, "tx { %s}", dbg);
            }
        }
        rs485->tx_mode(true);
        rs485->write_bytes(pkt->skb->priv.data, pkt->skb->len);
        rs485->tx_mode(false);

            // pretend that we received our own message

        bool txOpportunity = false;
        network_msg_t msg;

        ESP_LOGVV(TAG, "pretent rx: pkt typ=%s", enum_str(static_cast<datalink_typ_ctrl_t>(pkt->typ)));

        if (network_rx_msg(pkt, &msg, &txOpportunity) == ESP_OK) {

            ipc_send_network_msg_to_main_task(&msg, ipc);
        }
        free(pkt->skb);
        free((void *) pkt);
    }
}

/**
 * @brief FreeRTOS sub-task for periodic pool controller requests.
 *
 * Periodically sends heat and schedule request messages to the pool controller
 * to keep the pool state up to date.
 *
 * @param rs485_void Pointer to the RS-485 handle (as void* for FreeRTOS compatibility).
 */
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

/**
 * @brief FreeRTOS task for RS-485 communication and protocol handling.
 *
 * This function implements the main loop for the OPNpool component, responsible for:
 *   - Initializing the RS-485 interface and IPC structure.
 *   - Requesting initial controller information (version, time).
 *   - Spawning a periodic request sub-task for heat and schedule queries.
 *   - Continuously servicing requests from the main task and processing incoming RS-485
 *     packets.
 *   - Forwarding queued packets for transmission when a transmit opportunity is
 *     available.
 *   - Maintaining protocol state and relaying messages between the controller and main
 *     task.
 *
 * @param ipc_void Pointer to the IPC structure.
 */
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

    while (1) {

            // read from ipc->to_pool_q

        _service_requests_from_main(rs485, ipc);

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