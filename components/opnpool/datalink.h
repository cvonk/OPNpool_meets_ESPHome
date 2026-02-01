/**
 * @file datalink.h
 * @brief Data Link Layer Definitions for OPNpool component
 *
 * @details
 * This header defines the data structures, enumerations, and function prototypes for the
 * data link layer of the OPNpool component. The data link layer is responsible for
 * framing, parsing, and validating packets exchanged between the ESP32 and the Pentair
 * pool controller over RS-485. It enables reliable communication and protocol abstraction
 * for higher-level network and application logic.
 * 
 * The data link layer provides two functions:
 * 1. `datalink_rx_pkt()`: removes the header and tail of a RS-485 byte stream, verifies
 *    its integrity. 
 * 2. `datalink_create_pkt()`: adds the header and tail to create a RS-485 byte stream.
 *
 * The design supports multiple protocol variants (A5, IC) and hardware configurations,
 * and is intended for use in a single-threaded ESPHome environment.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

#ifndef PACK8
# define PACK8 __attribute__((aligned( __alignof__(uint8_t)), packed))
#endif

    // forward declarations (to avoid circular dependencies)
struct datalink_pkt_t;
struct rs485_instance_t;
using rs485_handle_t = rs485_instance_t *;
struct datalink_pkt_t;

    // 0x10 = suntouch ctrl system
    // 0x20 = easytouch
    // 0x21 = remote
    // 0x22 = wireless remote
    // 0x48 = quicktouch remote
    // 0x60 .. 0x6F = intelliflow pump 0 .. 15
    // 
    // `addrgroup` is the high nibble of the address

enum class datalink_group_addr_t : uint8_t {
  ALL    = 0x00,
  CTRL   = 0x01,
  REMOTE = 0x02,
  CHLOR  = 0x05,
  PUMP   = 0x06,
  X09    = 0x09
};

enum class datalink_dev_id_t : uint8_t {
    PRIMARY    = 0x00,
    SECONDARY  = 0x01,
    //REMOTE   = 0x02
};

using datalink_preamble_a5_t = uint8_t[3];
using datalink_preamble_ic_t = uint8_t[2];
using datalink_postamble_ic_t = uint8_t[2];

struct datalink_addr_t {
    uint8_t byte;  // lower nibble is device address; higher nibble is group address
    datalink_dev_id_t   get_dev_id()    const { return static_cast<datalink_dev_id_t>(byte & 0x0F); } 
    datalink_group_addr_t get_group_addr()  const { return static_cast<datalink_group_addr_t>((byte >> 4) & 0x0F); }    
    void set_dev_id(datalink_dev_id_t dev)      { byte = (byte & 0xF0) | (static_cast<uint8_t>(dev) & 0x0F); }
    void set_group_addr( datalink_group_addr_t grp) { byte = (byte & 0x0F) | (static_cast<uint8_t>(grp) << 4); }
} PACK8;
static_assert(sizeof(datalink_addr_t) == 1, "datalink_addr_t must be 1 byte");

struct datalink_hdr_ic_t {
    datalink_addr_t dst;  // destination address
    uint8_t         typ;  // message type
} PACK8;

struct datalink_hdr_a5_t {
    uint8_t         ver;  // protocol version id
    datalink_addr_t dst;  // destination address
    datalink_addr_t src;  // source address
    uint8_t         typ;  // message type
    uint8_t         len;  // # of data bytes following
} PACK8;

union datalink_hdr_t {
    datalink_hdr_ic_t ic;
    datalink_hdr_a5_t a5;
} PACK8;

struct datalink_head_a5_t {
    uint8_t                ff;
    datalink_preamble_a5_t preamble;
    datalink_hdr_a5_t      hdr;
} PACK8;

struct datalink_head_ic_t {
    uint8_t                ff;
    datalink_preamble_ic_t preamble;
    datalink_hdr_ic_t      hdr;
} PACK8;

/**
 * @brief Data link head union for protocol abstraction.
 *
 * This union provides access to the head fields for both IC and A5 protocol variants. It
 * enables unified handling of protocol-specific head data, including preamble and header,
 * for flexible packet parsing and construction.
 *
 * @var `ic`: Head structure for IC protocol packets (includes preamble and header).
 * @var `a5`: Head structure for A5 protocol packets (includes preamble and header).
 */
union datalink_head_t {
    datalink_head_ic_t ic;
    datalink_head_a5_t a5;
};

uint8_t const DATALINK_MAX_HEAD_SIZE = sizeof(datalink_head_t);

struct datalink_tail_a5_t {
    uint8_t  crc[2];
} PACK8;

struct datalink_tail_ic_t {
    uint8_t                 crc[1];
    datalink_postamble_ic_t postamble;
} PACK8;

/**
 * @brief Data link tail union for protocol abstraction.
 *
 * This union provides access to the tail fields for both IC and A5 protocol variants.
 * It enables unified handling of protocol-specific tail data, such as CRC and postamble,
 * for packet validation and parsing.
 * @var `ic`: Tail structure for IC protocol packets (includes CRC and postamble).
 * @var `a5`: Tail structure for A5 protocol packets (includes CRC).
 */
union datalink_tail_t {
    datalink_tail_ic_t ic;
    datalink_tail_a5_t a5;
};

uint8_t const DATALINK_MAX_TAIL_SIZE = sizeof(datalink_tail_t);

/**
 * @brief Data link layer function prototypes and constant declarations.
 *
 * These declarations provide the interface for key data link layer operations and
 * protocol constants:
 * - Address group and device address calculation
 * - CRC calculation for packet validation
 * - Protocol preamble and postamble constants for A5 and IC variants
 * - Packet receive and transmit functions for RS-485 communication
 */

datalink_group_addr_t datalink_group_addr(uint8_t const addr);
uint8_t datalink_device_id(uint8_t const addr);
datalink_addr_t datalink_dev_id(datalink_group_addr_t const group, datalink_dev_id_t const device_id);
uint16_t datalink_calc_crc(uint8_t const * const start, uint8_t const * const stop);
extern datalink_preamble_a5_t datalink_preamble_a5;
extern datalink_preamble_ic_t datalink_preamble_ic;
extern datalink_postamble_ic_t datalink_postamble_ic;

[[nodiscard]] esp_err_t datalink_rx_pkt(rs485_handle_t const rs485, datalink_pkt_t * const pkt);

void datalink_tx_pkt_queue(rs485_handle_t const rs485_handle, datalink_pkt_t const * const pkt);

} // namespace opnpool
} // namespace esphome
