/**
 * @file network_str.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Network layer: support enum/various to string
 * 
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * 
 * @details
 * This file provides string conversion utilities for the OPNpool network layer. It implements
 * helper functions to convert various protocol values, enums, and addresses into human-readable
 * strings for logging, debugging, and display purposes. These functions are used throughout
 * the OPNpool component to improve traceability and diagnostics.
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
 * SPDX-FileCopyrightText: Copyright 2014,2019,2022,2026 Coert Vonk
 */

#include <string.h>
#include <esp_system.h>
#include <esphome/core/log.h>

#include "utils.h"
#include "network.h"

namespace esphome {
namespace opnpool {

enum class network_pump_program_addr_t : uint16_t {
    UNKNOWN_2BF0 = 0x2BF0,
    UNKNOWN_02BF = 0x02BF,   
    PGM          = 0x02E4,  // program GPM
    RPM          = 0x02C4,  // program RPM
    EPRG         = 0x0321,  // select ext prog, 0x0000=P0, 0x0008=P1, 0x0010=P2, 0x0080=P3, 0x0020=P4
    ERPM0        = 0x0327,  // program ext program RPM0
    ERPM1        = 0x0328,  // program ext program RPM1
    ERPM2        = 0x0329,  // program ext program RPM2
    ERPM3        = 0x032A   // program ext program RPM3
};

char const *
network_ctrl_date_str(uint16_t const year, uint8_t const month, uint8_t const day)
{
    static char buffer[11];  // "YYYY-MM-DD\0"
    snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u", year, month, day);
    return buffer;
}

char const *
network_ctrl_time_str(uint8_t const hour, uint8_t const minute)
{
    static char buffer[6];  // "HH:MM\0"
    snprintf(buffer, sizeof(buffer), "%02u:%02u", hour, minute);
    return buffer;
}

/**
 * @brief                Get a string representation of the controller version number
 * 
 * @param major          The major version number
 * @param minor          The minor version number
 * @return char const *  A string representing the version number
 */

char const *
network_ctrl_version_str(uint8_t const major, uint8_t const minor)
{
    static char buffer[8];  // "MMM.mmm\0"
    snprintf(buffer, sizeof(buffer), "%u.%u", major, minor);
    return buffer;
}


/**
 * @brief Get a string representation of a pump program "address"
 * 
 * @param address        The "address" of the pump program
 * @return char const *  A string representing the pump program
 */
char const *
network_pump_program_str(uint16_t const address)
{
    auto prg_addr = static_cast<network_pump_program_addr_t>(address);
    
    switch (prg_addr) {
        case network_pump_program_addr_t::UNKNOWN_2BF0:  return "?2BF0";
        case network_pump_program_addr_t::UNKNOWN_02BF:  return "?02BF";
        case network_pump_program_addr_t::PGM:           return "pgm";
        case network_pump_program_addr_t::RPM:           return "rpm";
        case network_pump_program_addr_t::EPRG:          return "eprg";
        case network_pump_program_addr_t::ERPM0:         return "erpm0";
        case network_pump_program_addr_t::ERPM1:         return "erpm1";
        case network_pump_program_addr_t::ERPM2:         return "erpm2";
        case network_pump_program_addr_t::ERPM3:         return "erpm3";
        default: {
            static char hex_buffer[5];  // "XXXX\0"
            snprintf(hex_buffer, sizeof(hex_buffer), "%04x", address);
            return hex_buffer;
        }
    }
}

} // namespace opnpool
} // namespace esphome