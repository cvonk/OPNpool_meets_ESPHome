/**
 * @file datalink_rx.cpp
 * @brief Data Link layer: bytes from the RS485 transceiver to data packets
 *
 * @details
 * This file implements the data link layer receiver for the OPNpool component,
 * responsible for converting raw bytes from the RS485 transceiver into structured data
 * packets. It uses a state machine to detect protocol preambles, read packet headers,
 * data, and tails, and verify checksums for both A5 and IC protocols. The implementation
 * manages protocol-specific framing, handles CRC validation, and allocates socket buffers
 * for incoming packets. This layer ensures reliable and robust extraction of protocol
 * packets from the RS485 byte stream, providing validated data to higher-level network
 * processing in the OPNpool interface.
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
#include <esp_err.h>
#include <esphome/core/log.h>

#include "rs485.h"
#include "network.h"
#include "skb.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network_msg.h"

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

namespace esphome {
namespace opnpool {

static char const * const TAG = "datalink_rx";

uint8_t const DATALINK_MAX_DATA_SIZE = sizeof(network_msg_data_t);

struct proto_info_t  {
    uint8_t const * const  preamble;
#if 0    
    uint8_t const * const  postamble;
#endif
    uint8_t const          len;
    datalink_prot_t const  prot;
    uint8_t                idx;
};

static proto_info_t _proto_descr[] = {
    {
          .preamble = datalink_preamble_ic,
          .len = sizeof(datalink_preamble_ic),
          .prot = datalink_prot_t::IC,
          .idx = 0,
      },
    {
          .preamble = datalink_preamble_a5,
          .len = sizeof(datalink_preamble_a5),
          .prot = datalink_prot_t::A5_CTRL,  // distinction between A5_CTRL and A5_PUMP is based on src/dst in hdr
          .idx = 0,
    },
};

enum state_t {
    STATE_FIND_PREAMBLE,
    STATE_READ_HEAD,
    STATE_READ_DATA,
    STATE_READ_TAIL,
    STATE_CHECK_CRC,
    STATE_DONE,
};

struct local_data_t {
    size_t             head_len;
    size_t             tail_len;
    datalink_head_t *  head;
    datalink_tail_t *  tail;
    bool               crc_ok;
};

/**
 * @brief Reset the preamble match state for all supported protocols.
 *
 * Resets the internal state for all protocol preamble matchers, preparing them to
 * detect the start of a new packet in the RS-485 byte stream. Called at the beginning
 * of packet reception and after failed matches.
 */
static void
_preamble_reset()
{
    proto_info_t * info = _proto_descr;

    for (uint_least8_t ii = 0; ii < ARRAY_SIZE(_proto_descr); ii++, info++) {
        info->idx = 0;
    }
}

/**
 * @brief Check if the preamble for a protocol is complete given the next byte.
 *
 * Examines the next byte from the RS-485 stream to determine if it matches the expected
 * protocol preamble sequence. Advances the match index and sets a flag if the byte is
 * part of the preamble. Returns true if the full preamble is matched.
 *
 * @param pi               Protocol info structure.
 * @param b                Next byte from the stream.
 * @param part_of_preamble Set true if b matches part of the preamble.
 * @return                 True if preamble is complete, false otherwise.
 */
static bool
_preamble_complete(proto_info_t * const pi, uint8_t const b, bool * part_of_preamble)
{
    if (b == pi->preamble[pi->idx]) {
      *part_of_preamble = true;
      pi->idx++;
      if (pi->idx == pi->len) {
          return true;
      }
    } else {
          *part_of_preamble = false;
    }
    return false;
}

/**
 * @brief Waits until a valid A5/IC protocol preamble is received (or times-out).
 *
 * Reads bytes from the RS-485 interface until a valid preamble for either the A5 or IC
 * protocol is detected. Updates the packet structure with the detected protocol type and
 * stores the received preamble bytes in the local header buffer. Also sets header/tail
 * lengths for the detected protocol.
 *
 * @param rs485 RS485 handle.
 * @param local Local state for header/tail.
 * @param pkt   Packet structure to update.
 * @return      ESP_OK if preamble found, ESP_FAIL otherwise.
 */
static esp_err_t
_find_preamble(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt)
{
    uint8_t len = 0;
    uint8_t buf_size = 40;
    char dbg[buf_size];

    uint8_t byt;
    while (rs485->read_bytes(&byt, 1) == 1) {
        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
            len += snprintf(dbg + len, buf_size - len, " %02X", byt);
        }
        bool part_of_preamble = false;
        proto_info_t * info = _proto_descr;
        
        for (uint_least8_t ii = 0; !part_of_preamble && ii < ARRAY_SIZE(_proto_descr); ii++, info++) {
            if (_preamble_complete(info, byt, &part_of_preamble)) {
                ESP_LOGV(TAG, "%s (preamble)", dbg);
                pkt->prot = info->prot;
                uint8_t * preamble = NULL;
                switch (pkt->prot) {
                    case datalink_prot_t::A5_CTRL:
                    case datalink_prot_t::A5_PUMP:
                        // add to pkt just in case we want to retransmit it
                        local->head->a5.ff = 0xFF;
                        preamble = local->head->a5.preamble;
                        local->head_len = sizeof(datalink_head_a5_t) ;
                        local->tail_len = sizeof(datalink_tail_a5_t) ;
                        break;
                    case datalink_prot_t::IC:
                        preamble = local->head->ic.preamble;
                        local->head_len = sizeof(datalink_head_ic_t) ;
                        local->tail_len = sizeof(datalink_tail_ic_t) ;
                        break;
                    default:
                        return ESP_FAIL;
                }
                for (uint_least8_t jj = 0; jj < info->len; jj++) {
                    preamble[jj] = info->preamble[jj];
                }
                _preamble_reset();
                return ESP_OK;
            }
        }

        if (!part_of_preamble) {  // could be the beginning of the next
            _preamble_reset();
            proto_info_t * info = _proto_descr;

            for (uint_least8_t ii = 0; ii < ARRAY_SIZE(_proto_descr); ii++, info++) {
                (void)_preamble_complete(info, byt, &part_of_preamble);
            }
        }
    }
    return ESP_FAIL;
}


/**
 * @brief        Returns the length of the IC network message for a given type.
 *
 * Looks up the expected length of the IC protocol network message for the given type.
 * This is required because the datalink_typ_chlor_t enum is non-continuous.
 *
 * @param ic_typ The IC message type (as uint8_t/datalink_typ_chlor_t).
 * @return       The size of the corresponding network message struct, or 0 if unknown.
 */
static uint8_t _network_ic_len(uint8_t const ic_typ)
{
    auto typ = static_cast<datalink_typ_chlor_t>(ic_typ);
    static const struct {
        datalink_typ_chlor_t typ;
        uint8_t len;
    } type_lut[] = {
        {datalink_typ_chlor_t::PING_REQ,   sizeof(network_msg_chlor_ping_req_t)},
        {datalink_typ_chlor_t::PING_RESP,  sizeof(network_msg_chlor_ping_resp_t)},
        {datalink_typ_chlor_t::NAME_RESP,  sizeof(network_msg_chlor_name_resp_t)},
        {datalink_typ_chlor_t::LEVEL_SET,  sizeof(network_msg_chlor_level_set_t)},
        {datalink_typ_chlor_t::LEVEL_RESP, sizeof(network_msg_chlor_level_resp_t)},
        {datalink_typ_chlor_t::NAME_REQ,   sizeof(network_msg_chlor_name_req_t)},
    };
    for (const auto& entry : type_lut) {
        if (entry.typ == typ) return entry.len;
    }
    return 0;
}

/**
 * @brief Reads a A5/IC protocol header (or times-out).
 *
 * Reads the header portion of a detected A5 or IC protocol packet from the RS-485 bus.
 * Populates the packet structure with type, source, destination, and data length fields.
 *
 * @param rs485 RS485 handle.
 * @param local Local state for header/tail.
 * @param pkt   Packet structure to update.
 * @return      ESP_OK if header read, ESP_FAIL otherwise.
 */
static esp_err_t
_read_head(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt)
{
    switch (pkt->prot) {
        case datalink_prot_t::A5_CTRL:
        case datalink_prot_t::A5_PUMP: {
            datalink_hdr_a5_t * const hdr = &local->head->a5.hdr;

            if (rs485->read_bytes((uint8_t *) hdr, sizeof(datalink_hdr_a5_t)) == sizeof(datalink_hdr_a5_t)) {

                ESP_LOGV(TAG, " %02X %02X %02X %02X %02X (header)", hdr->ver, hdr->dst, hdr->src, hdr->typ, hdr->len);

                if (hdr->len > DATALINK_MAX_DATA_SIZE) {
                  return ESP_FAIL;  // pkt length exceeds what we have planned for
                }
                if ( (datalink_groupaddr(hdr->src) == datalink_addrgroup_t::PUMP) ||
                     (datalink_groupaddr(hdr->dst) == datalink_addrgroup_t::PUMP) ) {
                    pkt->prot = datalink_prot_t::A5_PUMP;
                }
                pkt->typ.raw  = hdr->typ;
                pkt->src      = hdr->src;
                pkt->dst      = hdr->dst;
                pkt->data_len = hdr->len;
                if (pkt->data_len > sizeof(network_msg_data_a5_t)) {
                    return ESP_FAIL;
                }
                return ESP_OK;
            }
            break;
        }
        case datalink_prot_t::IC: {
            datalink_hdr_ic_t * const hdr = &local->head->ic.hdr;
            
            if (rs485->read_bytes((uint8_t *) hdr, sizeof(datalink_hdr_ic_t)) == sizeof(datalink_hdr_ic_t)) {
                ESP_LOGV(TAG, " %02X %02X (header)", hdr->dst, hdr->typ);

                pkt->typ.raw  = hdr->typ;
                pkt->src      = 0;
                pkt->dst      = hdr->dst;
                pkt->data_len = _network_ic_len(hdr->typ);
                return ESP_OK;
            }
            break;
        }
        default:
            break;
    }
    ESP_LOGW(TAG, "unsupported pkt->prot 0x%02X", static_cast<uint8_t>(pkt->prot));
      return ESP_FAIL;
}

/**
 * @brief Reads the data payload of a previously detected A5 or IC protocol packet.
 *
 * Reads the data section from the RS-485 bus and stores it in the packet's data buffer.
 * Called after the header has been successfully read.
 *
 * @param rs485 RS485 handle.
 * @param local Local state for header/tail.
 * @param pkt   Packet structure to update.
 * @return      ESP_OK if data read, ESP_FAIL otherwise.
 */
static esp_err_t
_read_data(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt)
{
    uint8_t len = 0;
    uint8_t buf_size = 100;
    char buf[buf_size]; *buf = '\0';

    if (rs485->read_bytes((uint8_t *) pkt->data, pkt->data_len) == pkt->data_len) {
        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
            for (uint_least8_t ii = 0; ii < pkt->data_len; ii++) {
                len += snprintf(buf + len, buf_size - len, " %02X", pkt->data[ii]);
            }
            ESP_LOGV(TAG, "%s (data)", buf);
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

/**
 * @brief Reads the tail (CRC and postamble) of a previously detected A5 or IC protocol packet.
 *
 * Reads the tail section from the RS-485 bus, which contains the CRC and, for IC protocol,
 * the postamble. Stores the received bytes in the local tail buffer.
 *
 * @param rs485 RS485 handle.
 * @param local Local state for header/tail.
 * @param pkt   Packet structure to update.
 * @return      ESP_OK if tail read, ESP_FAIL otherwise.
 */
static esp_err_t
_read_tail(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt)
{
    switch (pkt->prot) {
        case datalink_prot_t::A5_CTRL:
        case datalink_prot_t::A5_PUMP: {
            uint8_t * const crc = local->tail->a5.crc;
            if (rs485->read_bytes(crc, sizeof(datalink_tail_a5_t)) == sizeof(datalink_tail_a5_t)) {
                ESP_LOGV(TAG, " %03X (checksum)", (uint16_t)crc[0] << 8 | crc[1]);
                return ESP_OK;
            }
            break;
        }
        case datalink_prot_t::IC: {
            uint8_t * const crc = local->tail->ic.crc;
            uint8_t * const postamble = local->tail->ic.postamble;
            if (rs485->read_bytes(crc, sizeof(datalink_tail_ic_t)) == sizeof(datalink_tail_ic_t)) {
                ESP_LOGV(TAG, " %02X (checksum)", crc[0]);
                ESP_LOGV(TAG, " %02X %02X (postamble)", postamble[0], postamble[1]);
                return ESP_OK;
            }
            break;
        }
        default:
            break;
    }
    
    ESP_LOGW(TAG, "unsupported pkt->prot 0x%02X !", static_cast<uint8_t>(pkt->prot));
    return ESP_FAIL;
}

/**
 * @brief Check the CRC of the received packet.
 *
 * Verifies the CRC of the received packet by comparing the received CRC value with the
 * calculated CRC over the packet's contents. Updates the local CRC status and returns
 * the result.
 *
 * @param rs485 RS485 handle.
 * @param local Local state for header/tail.
 * @param pkt   Packet structure to update.
 * @return      ESP_OK if CRC matches, ESP_FAIL otherwise.
 */
static esp_err_t
_check_crc(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt)
{
    struct {uint16_t rx, calc;} crc;

    switch (pkt->prot) {
        case datalink_prot_t::A5_CTRL:
        case datalink_prot_t::A5_PUMP: {
            crc.rx = (uint16_t)local->tail->a5.crc[0] << 8 | local->tail->a5.crc[1];
            uint8_t * const crc_start = &local->head->a5.preamble[sizeof(datalink_preamble_a5_t) - 1];  // starting at the last byte of the preamble
            uint8_t * const crc_stop = pkt->data + pkt->data_len;
            crc.calc = datalink_calc_crc(crc_start, crc_stop);
            break;
        }
        case datalink_prot_t::IC: {
            crc.rx = local->tail->ic.crc[0];
            uint8_t * const crc_start = local->head->ic.preamble;  // starting at the first byte of the preamble
            uint8_t * const crc_stop = pkt->data + pkt->data_len;
            crc.calc = datalink_calc_crc(crc_start, crc_stop) & 0xFF;
            break;
        }
        default:
            return ESP_FAIL;
    }

    local->crc_ok = crc.rx == crc.calc;
    if (local->crc_ok) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "crc err (rx=0x%03x calc=0x%03x)", crc.rx, crc.calc);
    return ESP_FAIL;
}

using state_fnc_t = esp_err_t (*)(rs485_handle_t const rs485, local_data_t * const local, datalink_pkt_t * const pkt);

struct state_transition_t {
    state_t     state;
    state_fnc_t fnc;
    state_t     on_ok;
    state_t     on_err;
};

static state_transition_t state_transitions[] = {
    { STATE_FIND_PREAMBLE, _find_preamble, STATE_READ_HEAD,     STATE_FIND_PREAMBLE },
    { STATE_READ_HEAD,     _read_head,     STATE_READ_DATA,     STATE_FIND_PREAMBLE },
    { STATE_READ_DATA,     _read_data,     STATE_READ_TAIL,     STATE_FIND_PREAMBLE },
    { STATE_READ_TAIL,     _read_tail,     STATE_CHECK_CRC,     STATE_FIND_PREAMBLE },
    { STATE_CHECK_CRC,     _check_crc,     STATE_DONE,          STATE_FIND_PREAMBLE },
};

/**
 * @brief Receive a protocol packet from the RS-485 bus using a state machine.
 *
 * This function implements the main receive loop for the data link layer. It uses a state
 * machine (see `state_transitions[]`) to detect protocol preambles, read packet headers,
 * payloads, and tails, and verify checksums for supported protocols (A5 and IC). The
 * function allocates a socket buffer, extracts and validates the packet, and returns the
 * result to the caller.
 *
 * Called from `pool_task` to process incoming RS-485 data and convert it into structured
 * packets for higher-level network processing.
 *
 * @param rs485 RS485 handle for reading bytes from the bus.
 * @param pkt   Pointer to a packet structure to fill with received data.
 * @return      ESP_OK if a valid packet is received and CRC matches, ESP_FAIL otherwise.
 */
esp_err_t
datalink_rx_pkt(rs485_handle_t const rs485, datalink_pkt_t * const pkt)
{
    state_t state = STATE_FIND_PREAMBLE;
    pkt->skb = skb_alloc(DATALINK_MAX_HEAD_SIZE + DATALINK_MAX_DATA_SIZE + DATALINK_MAX_TAIL_SIZE);
    local_data_t local;
    local.head = (datalink_head_t *) skb_put(pkt->skb, DATALINK_MAX_HEAD_SIZE);

    while (1) {
        state_transition_t * transition = state_transitions;
        for (uint_least8_t ii = 0; ii < ARRAY_SIZE(state_transitions); ii++, transition++) {
            if (state == transition->state) {

                    // calls the registered function for the current state. it will store
                    // head/tail in `local` and update `pkt`

                bool const ok = transition->fnc(rs485, &local, pkt) == ESP_OK;

                    // find the new state

                state_t const new_state = ok ? transition->on_ok : transition->on_err;

                    // claim socket buffers to store the bytes received

                switch (new_state) {
                    case STATE_FIND_PREAMBLE:
                        skb_reset(pkt->skb);
                        local.head = (datalink_head_t *) skb_put(pkt->skb, DATALINK_MAX_HEAD_SIZE);
                        break;
                    case STATE_READ_HEAD:
                        skb_call(pkt->skb, DATALINK_MAX_HEAD_SIZE - local.head_len);  // release unused bytes
                        break;
                    case STATE_READ_DATA:
                        pkt->data = (datalink_data_t *) skb_put(pkt->skb, pkt->data_len);
                        break;
                    case STATE_READ_TAIL:
                        local.tail = (datalink_tail_t *) skb_put(pkt->skb, local.tail_len);
                        break;
                    case STATE_CHECK_CRC:
                        break;
                    case STATE_DONE:
                        if (!local.crc_ok) {
                            free(pkt->skb);
                            return ESP_FAIL;
                        }
                        return ESP_OK;
                        break;
                }
                state = new_state;
            }
        }
    }
}

} // namespace opnpool
} // namespace esphome