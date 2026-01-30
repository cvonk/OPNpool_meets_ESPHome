/**
 * @file network_create.cpp
 * @brief Network layer: create datalink_pkt from network_msg
 *
 * @details
 * This file implements the logic for constructing datalink packets from higher-level
 * network messages within the OPNpool component. It provides functions to allocate
 * buffers, set protocol headers, and serialize network message data into the appropriate
 * datalink packet format for transmission over RS-485.
 *
 * ESPHome operates in a single-threaded environment, so explicit thread safety measures
 * are not required within the pool_task context.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include <string.h>
#include <esphome/core/log.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "skb.h"
#include "network.h"
#include "network_msg.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

constexpr char TAG[] = "network_create";

/*
 * Create datalink_pkt from network_msg.
 */

esp_err_t
network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt)
{
        // get protocol info from the lookup table network_msg_typ_info[] in network_msg.h
    const network_typ_info_t * info = network_msg_typ_get_info(msg->typ);
    if (info == nullptr) {
        ESP_LOGE(TAG, "unknown msg typ(%s)", enum_str(msg->typ));
        return ESP_FAIL;
    }

    uint32_t data_len = info->size;

#if 0
    size_t data_len;
    if (network_msg_typ_get_size(msg->typ, &data_len) != ESP_OK) {
        ESP_LOGE(TAG, "unknown msg typ(%s)", enum_str(msg->typ));
        return ESP_FAIL;
    }
#endif

    pkt->prot = info->proto;
    pkt->typ = info->datalink_typ;
    pkt->data_len = data_len;
    pkt->skb = skb_alloc(DATALINK_MAX_HEAD_SIZE + data_len + DATALINK_MAX_TAIL_SIZE);
    skb_reserve(pkt->skb, DATALINK_MAX_HEAD_SIZE);
    pkt->data = skb_put(pkt->skb, data_len);
    memcpy(pkt->data, msg->u.raw, data_len);    
    return ESP_OK;
}

} // namespace opnpool
} // namespace esphome