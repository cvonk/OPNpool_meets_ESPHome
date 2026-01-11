/**
 * @brief OPNpool - Network layer: creates network_msg from datalink_pkt and visa versa
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
#include <esp_types.h>
#include <esphome/core/log.h>

#include "datalink.h"
#include "network.h"

namespace esphome {
namespace opnpool {

// static char const * const TAG = "network";

uint8_t
network_ic_len(uint8_t const ic_typ)
{
    auto typ = static_cast<datalink_typ_chlor_t>(ic_typ);

    switch (typ) {
        case datalink_typ_chlor_t::PING_REQ:
            return sizeof(network_msg_chlor_ping_req_t);
        case datalink_typ_chlor_t::PING_RESP:
            return sizeof(network_msg_chlor_ping_resp_t);
        case datalink_typ_chlor_t::NAME_RESP:
            return sizeof(network_msg_chlor_name_resp_t);
        case datalink_typ_chlor_t::LEVEL_SET:
            return sizeof(network_msg_chlor_level_set_t);
        case datalink_typ_chlor_t::LEVEL_RESP:
            return sizeof(network_msg_chlor_level_resp_t);  // was ..ping_resp_t
        case datalink_typ_chlor_t::NAME_REQ:
            return sizeof(network_msg_chlor_name_req_t);
        default:
            return 0;
    };
}

} // namespace opnpool
} // namespace esphome