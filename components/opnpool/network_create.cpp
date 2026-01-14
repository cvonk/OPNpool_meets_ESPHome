/**
 * @file network_create.cpp
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Network layer: create datalink_pkt from network_msg
 * 
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
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

#include "datalink.h"
#include "datalink_pkt.h"
#include "utils.h"
#include "skb.h"
#include "network.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "network_create";

skb_handle_t
_skb_alloc_a5(size_t const msg_size)
{
    skb_handle_t const txb = skb_alloc(sizeof(datalink_head_a5_t) + msg_size + sizeof(datalink_tail_a5_t));
    skb_reserve(txb, sizeof(datalink_head_a5_t));
    return txb;
}

/*
 * Create datalink_pkt from network_msg.
 */

esp_err_t
network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt)
{
        // get protocol info from the lookup table network_msg_typ_info[] in network_msg.h
    const network_msg_typ_info_t * info = network_msg_typ_get_info(msg->typ);
    if (info == nullptr) {
        ESP_LOGE(TAG, "unknown msg typ(%s)", network_msg_typ_str(msg->typ));
        return ESP_FAIL;
    }

        // get message size from the lookup table network_msg_typ_sizes[] in network_msg.h
    size_t data_len;
    if (network_msg_typ_get_size(msg->typ, &data_len) != ESP_OK) {
        ESP_LOGE(TAG, "unknown msg typ(%s)", network_msg_typ_str(msg->typ));
        return ESP_FAIL;
    }

    pkt->prot = info->proto;
    pkt->typ = info->typ;
    pkt->data_len = data_len;
    pkt->skb = skb_alloc(DATALINK_MAX_HEAD_SIZE + data_len + DATALINK_MAX_TAIL_SIZE);
    skb_reserve(pkt->skb, DATALINK_MAX_HEAD_SIZE);
    pkt->data = skb_put(pkt->skb, data_len);
    memcpy(pkt->data, msg->u.bytes, data_len);    
    return ESP_OK;
}

} // namespace opnpool
} // namespace esphome