/**
 * @file datalink_tx.cpp
 * @brief Data Link layer: bytes from the RS485 transceiver from data packets
 *
 * @details
 * This file implements the data link layer transmitter for the OPNpool component,
 * responsible for constructing protocol-compliant packets for transmission over the RS485
 * bus. It provides functions to add protocol-specific headers and tails (including
 * preambles and CRCs) for both A5 and IC protocols, ensuring correct framing and
 * integrity of outgoing messages. The implementation manages buffer manipulation,
 * protocol selection, and queues completed packets for transmission by the RS485 driver.
 * This layer enables reliable and standards-compliant communication with pool equipment
 * by encapsulating higher-level messages into properly formatted data link packets.
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
#include <esphome/core/log.h>

#include "rs485.h"
#include "skb.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "enum_helpers.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "datalink_tx";


/**
 * @brief      Fills the IC protocol packet header fields for transmission.
 *
 * @param head Pointer to the IC protocol header structure to fill.
 * @param txb  Socket buffer handle for the outgoing packet.
 * @param typ  Message type union for the packet.
 */
static void
_enter_ic_head(datalink_head_ic_t * const head, skb_handle_t const txb, datalink_typ_t const typ)
{
    head->ff = 0xFF;
    for (uint_least8_t ii = 0; ii < sizeof(datalink_preamble_ic); ii++) {
        head->preamble[ii] = datalink_preamble_ic[ii];
    }
    head->hdr.dst = datalink_devaddr(datalink_addrgroup_t::CTRL, 0);
    head->hdr.typ = typ.raw;
}


/**
 * @brief       Fills the IC protocol packet tail (CRC) for transmission.
 *
 * @param tail  Pointer to the IC protocol tail structure to fill.
 * @param start Pointer to the start of the data for CRC calculation.
 * @param stop  Pointer to the end of the data for CRC calculation.
 */
static void
_enter_ic_tail(datalink_tail_ic_t * const tail, uint8_t const * const start, uint8_t const * const stop)
{
    tail->crc[0] = (uint8_t) datalink_calc_crc(start, stop);
}


/**
 * @brief          Fills the A5 protocol packet header fields for transmission.
 *
 * @param head     Pointer to the A5 protocol header structure to fill.
 * @param txb      Socket buffer handle for the outgoing packet.
 * @param typ      Message type union for the packet.
 * @param data_len Length of the data payload.
 */
static void
_enter_a5_head(datalink_head_a5_t * const head, skb_handle_t const txb, datalink_typ_t const typ, size_t const data_len)
{
    head->ff = 0xFF;
    for (uint_least8_t ii = 0; ii < sizeof(datalink_preamble_a5); ii++) {
        head->preamble[ii] = datalink_preamble_a5[ii];
    }
    head->hdr.ver = 0x01;
    head->hdr.dst = datalink_devaddr(datalink_addrgroup_t::CTRL, 0);
    head->hdr.src = datalink_devaddr(datalink_addrgroup_t::REMOTE, 2);  // 2BD 0x20 is the wired remote; 0x22 is the wireless remote (Screen Logic, or any app)
    head->hdr.typ = typ.raw;
    head->hdr.len = data_len;
}


/**
 * @brief       Fills the A5 protocol packet tail (CRC) for transmission.
 *
 * @param tail  Pointer to the A5 protocol tail structure to fill.
 * @param start Pointer to the start of the data for CRC calculation.
 * @param stop  Pointer to the end of the data for CRC calculation.
 */
static void
_enter_a5_tail(datalink_tail_a5_t * const tail, uint8_t const * const start, uint8_t const * const stop)
{
    uint16_t crcVal = datalink_calc_crc(start, stop);
    tail->crc[0] = crcVal >> 8;
    tail->crc[1] = crcVal & 0xFF;
}


/**
 * @brief
 * Adds protocol headers and tails to a data packet and queues it for RS485 transmission.
 *
 * @details
 * This function constructs a protocol-compliant packet by adding the appropriate header
 * and tail (preamble and CRC) for the specified protocol (IC, A5/controller, or A5/pump).
 * It prepares the socket buffer for transmission, and enqueues the completed packet for
 * transmission by the RS485 driver. This function is typically called from the pool_task
 * to send messages to pool equipment.
 *
 * @param rs485 Pointer to the RS485 interface handle.
 * @param pkt   Pointer to the datalink packet structure to be transmitted.
 */
void
datalink_tx_pkt_queue(rs485_handle_t const rs485, datalink_pkt_t const * const pkt)
{
    skb_handle_t const skb = pkt->skb;

    switch (pkt->prot) {
        case datalink_prot_t::IC: {
            datalink_head_ic_t * const head = (datalink_head_ic_t *) skb_push(skb, sizeof(datalink_head_ic_t));
            _enter_ic_head(head, skb, pkt->typ);

            uint8_t * crc_start = head->preamble;
            uint8_t * crc_stop = skb->priv.tail;
            datalink_tail_ic_t * const tail = (datalink_tail_ic_t *) skb_put(skb, sizeof(datalink_tail_ic_t));
            _enter_ic_tail(tail, crc_start, crc_stop);
            break;
        }
        case datalink_prot_t::A5_CTRL:
        case datalink_prot_t::A5_PUMP: {
            datalink_head_a5_t * const head = (datalink_head_a5_t *) skb_push(skb, sizeof(datalink_head_a5_t));
            _enter_a5_head(head, skb, pkt->typ, pkt->data_len);

            uint8_t * crc_start = head->preamble + sizeof(datalink_preamble_a5_t) - 1;
            uint8_t * crc_stop = skb->priv.tail;
            datalink_tail_a5_t * const tail = (datalink_tail_a5_t *) skb_put(skb, sizeof(datalink_tail_a5_t));
            _enter_a5_tail(tail, crc_start, crc_stop);
            break;
        }
        default: {
            ESP_LOGE(TAG, "Unsupported protocol type: %02X", static_cast<uint8_t>(pkt->prot));
        }
    }
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
        size_t const dbg_size = 128;
        char dbg[dbg_size];
        (void) skb_print(TAG, skb, dbg, dbg_size);
        ESP_LOGV(TAG, " %s: { %s}", enum_str(pkt->prot), dbg);
    }

        // queue for transmission by `pool_task`
    rs485->queue(rs485, pkt);
}

} // namespace opnpool
} // namespace esphome