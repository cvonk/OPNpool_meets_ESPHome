/**
 * @brief OPNpool - Network layer: support enum/various to string
 *
 * © Copyright 2014, 2019, 2022, 2026, Coert Vonk
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

char const *
network_date_str(uint8_t const year, uint8_t const month, uint8_t const day)
{
    uint_least8_t const nrdigits = 10; // e.g. 2015-12-31

    if (name_str.idx + nrdigits + 1U >= ARRAY_SIZE(name_str.str)) {
      return name_str.noMem;  // increase size of str.str[]
    }
    char * s = name_str.str + name_str.idx;
    s[0] = '2'; s[1] = '0';
    s[2] = name_str.digits[year / 10];
    s[3] = name_str.digits[year % 10];
    s[4] = s[7] = '-';
    s[5] = name_str.digits[month / 10];
    s[6] = name_str.digits[month % 10];
    s[8] = name_str.digits[day / 10];
    s[9] = name_str.digits[day % 10];
    s[nrdigits] = '\0';
    name_str.idx += nrdigits + 1U;
    return s;
}

char const *
network_time_str(uint8_t const hours, uint8_t const minutes)
{
	uint8_t const nrdigits = 5;  // 00:00

	if (name_str.idx + nrdigits + 1U >= ARRAY_SIZE(name_str.str)) {
		return name_str.noMem;  // increase size of str.str[]
	}
	char * s = name_str.str + name_str.idx;
	s[0] = name_str.digits[hours / 10];
	s[1] = name_str.digits[hours % 10];
	s[2] = ':';
	s[3] = name_str.digits[minutes / 10];
	s[4] = name_str.digits[minutes % 10];
	s[nrdigits] = '\0';
	name_str.idx += nrdigits + 1U;
	return s;
}

char const *
network_version_str(uint8_t const major, uint8_t const minor)
{
	uint8_t const nrdigits = 6;  // v2.080

	if (name_str.idx + nrdigits + 1U >= ARRAY_SIZE(name_str.str)) {
		return name_str.noMem;  // increase size of str.str[]
	}
	char * s = name_str.str + name_str.idx;
	s[0] = 'v';
	s[1] = name_str.digits[major % 10];
	s[2] = '.';
	s[3] = '0';
	s[4] = name_str.digits[minor / 10];
	s[5] = name_str.digits[minor % 10];
	s[nrdigits] = '\0';
	name_str.idx += nrdigits + 1U;
	return s;
}

/**
 * @brief Get a string representation of a pump program "address"
 * 
 * @param address        The "address" of the pump program
 * @return char const *  A string representing the pump program
 */
char const *
network_pump_prg_str(uint16_t const address)
{
    auto prg_addr = static_cast<network_pump_prg_addr_t>(address);
    
    switch (prg_addr) {
        case network_pump_prg_addr_t::UNKNOWN:  return "?";
        case network_pump_prg_addr_t::PGM:      return "pgm";
        case network_pump_prg_addr_t::RPM:      return "rpm";
        case network_pump_prg_addr_t::EPRG:     return "eprg";
        case network_pump_prg_addr_t::ERPM0:    return "erpm0";
        case network_pump_prg_addr_t::ERPM1:    return "erpm1";
        case network_pump_prg_addr_t::ERPM2:    return "erpm2";
        case network_pump_prg_addr_t::ERPM3:    return "erpm3";
        default: {
            static char hex_buffer[7];  // "0xXXXX\0"
            snprintf(hex_buffer, sizeof(hex_buffer), "0x%04x", address);
            return hex_buffer;
        }
    }
}

} // namespace opnpool
} // namespace esphome