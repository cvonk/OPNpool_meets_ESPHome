/**
 * @file datalink.cpp
 * @brief Data Link layer: bytes from the RS485 transceiver to/from data packets
 *
 * @details
 * This file implements core functions for the OPNpool data link layer, facilitating the
 * conversion between raw RS485 byte streams and structured protocol data packets. It
 * provides utilities for handling protocol preambles and postambles, address group
 * extraction and composition, and CRC calculation for packet integrity. These
 * foundational routines are used by both the transmitter and receiver to ensure reliable
 * and standards- compliant communication between the ESPHome component and pool equipment
 * over the RS485 bus.
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

#include "datalink.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

// constexpr char TAG[] = "datalink";

datalink_preamble_a5_t datalink_preamble_a5  = { 0x00, 0xFF, 0xA5 };  // use of 0xA5 in the preamble makes the detection more reliable
datalink_preamble_ic_t datalink_preamble_ic  = { 0x10, 0x02 };
datalink_preamble_ic_t datalink_postamble_ic = { 0x10, 0x03 };

/**
 * @brief      Extracts the address group from a 16-bit A5 address.
 *
 * @param addr The full 16-bit address.
 * @return     The address group as a datalink_addrgroup_t.
 */
datalink_addrgroup_t
datalink_addr_group(uint16_t const addr)
{
    return static_cast<datalink_addrgroup_t>((addr >> 4) & 0x0F);
}


/**
 * @brief      Extracts the address id from a 16-bit A5 address.
 *
 * @param addr The full 16-bit address.
 * @return     The address id as a uint8_t.
 */
uint8_t
datalink_device_id(uint16_t const addr)
{
    return static_cast<uint8_t>(addr & 0x0F);
}


/**
 * @brief       Composes a device address from an address group and device ID.
 *
 * @param group The address group.
 * @param id    The device ID within the group.
 * @return      The composed 8-bit device address.
 */
uint8_t
datalink_devaddr(datalink_addrgroup_t const group, uint8_t const device_id)
{
    return (static_cast<uint8_t>(group) << 4) | (device_id & 0x0F);
}

/**
 * @brief       Calculates the CRC for a data buffer.
 *
 * @param start Pointer to the start of the data buffer.
 * @param stop  Pointer to one past the end of the data buffer.
 * @return      The calculated 16-bit CRC value.
 */
uint16_t
datalink_calc_crc(uint8_t const * const start, uint8_t const * const stop)
{
    uint16_t crc = 0;
    for (uint8_t const * byte = start; byte < stop; byte++) {
        crc += *byte;
    }
    return crc;
}

} // namespace opnpool
} // namespace esphome