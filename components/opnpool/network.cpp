/**
 * @file network.cpp
 * @brief OPNpool - Network layer: creates network_msg from datalink_pkt and visa versa
 * 
 * Thread safety is not provided, because it is not required for the single-threaded nature of ESPHome.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
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