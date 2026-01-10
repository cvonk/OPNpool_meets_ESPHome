/**
 * @brief OPNpool - Network layer: create datalink_pkt from network_msg
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

// Size lookup table for message types (moved from X-macro)
static constexpr size_t network_msg_typ_sizes[] = {
    0,  // 0: NONE
    sizeof(network_msg_ctrl_set_ack_t),  // 1: CTRL_SET_ACK
    sizeof(network_msg_ctrl_circuit_set_t),  // 2: CTRL_CIRCUIT_SET
    sizeof(network_msg_ctrl_sched_req_t),  // 3: CTRL_SCHED_REQ
    sizeof(network_msg_ctrl_sched_resp_t),  // 4: CTRL_SCHED_RESP
    sizeof(network_msg_ctrl_state_bcast_t),  // 5: CTRL_STATE_BCAST
    sizeof(network_msg_ctrl_time_req_t),  // 6: CTRL_TIME_REQ
    sizeof(network_msg_ctrl_time_resp_t),  // 7: CTRL_TIME_RESP
    sizeof(network_msg_ctrl_time_set_t),  // 8: CTRL_TIME_SET
    sizeof(network_msg_ctrl_heat_req_t),  // 9: CTRL_HEAT_REQ
    sizeof(network_msg_ctrl_heat_resp_t),  // 10: CTRL_HEAT_RESP
    sizeof(network_msg_ctrl_heat_set_t),  // 11: CTRL_HEAT_SET
    sizeof(network_msg_ctrl_layout_req_t),  // 12: CTRL_LAYOUT_REQ
    sizeof(network_msg_ctrl_layout_resp_t),  // 13: CTRL_LAYOUT_RESP
    sizeof(network_msg_ctrl_layout_set_t),  // 14: CTRL_LAYOUT_SET
    sizeof(network_msg_pump_reg_set_t),  // 15: PUMP_REG_SET
    sizeof(network_msg_pump_reg_resp_t),  // 16: PUMP_REG_RESP
    sizeof(network_msg_pump_ctrl_t),  // 17: PUMP_CTRL_SET
    sizeof(network_msg_pump_ctrl_t),  // 18: PUMP_CTRL_RESP
    sizeof(network_msg_pump_mode_t),  // 19: PUMP_MODE_SET
    sizeof(network_msg_pump_mode_t),  // 20: PUMP_MODE_RESP
    sizeof(network_msg_pump_run_t),  // 21: PUMP_RUN_SET
    sizeof(network_msg_pump_run_t),  // 22: PUMP_RUN_RESP
    sizeof(network_msg_pump_status_req_t),  // 23: PUMP_STATUS_REQ
    sizeof(network_msg_pump_status_resp_t),  // 24: PUMP_STATUS_RESP
    sizeof(network_msg_chlor_ping_req_t),  // 25: CHLOR_PING_REQ
    sizeof(network_msg_chlor_ping_resp_t),  // 26: CHLOR_PING_RESP
    sizeof(network_msg_chlor_name_resp_t),  // 27: CHLOR_NAME_RESP
    sizeof(network_msg_chlor_level_set_t),  // 28: CHLOR_LEVEL_SET
    sizeof(network_msg_chlor_level_resp_t),  // 29: CHLOR_LEVEL_RESP
    sizeof(network_msg_chlor_name_req_t),  // 30: CHLOR_NAME_REQ
    sizeof(network_msg_ctrl_valve_req_t),  // 31: CTRL_VALVE_REQ
    sizeof(network_msg_ctrl_valve_resp_t),  // 32: CTRL_VALVE_RESP
    sizeof(network_msg_ctrl_version_req_t),  // 33: CTRL_VERSION_REQ
    sizeof(network_msg_ctrl_version_resp_t),  // 34: CTRL_VERSION_RESP
    sizeof(network_msg_ctrl_solarpump_req_t),  // 35: CTRL_SOLARPUMP_REQ
    sizeof(network_msg_ctrl_solarpump_resp_t),  // 36: CTRL_SOLARPUMP_RESP
    sizeof(network_msg_ctrl_delay_req_t),  // 37: CTRL_DELAY_REQ
    sizeof(network_msg_ctrl_delay_resp_t),  // 38: CTRL_DELAY_RESP
    sizeof(network_msg_ctrl_heat_setpt_req_t),  // 39: CTRL_HEAT_SETPT_REQ
    sizeof(network_msg_ctrl_heat_setpt_resp_t),  // 40: CTRL_HEAT_SETPT_RESP
    sizeof(network_msg_ctrl_circ_names_req_t),  // 41: CTRL_CIRC_NAMES_REQ
    sizeof(network_msg_ctrl_circ_names_resp_t),  // 42: CTRL_CIRC_NAMES_RESP
    sizeof(network_msg_ctrl_scheds_req_t),  // 43: CTRL_SCHEDS_REQ
    sizeof(network_msg_ctrl_scheds_resp_t),  // 44: CTRL_SCHEDS_RESP
    sizeof(network_msg_ctrl_chem_req_t),  // 45: CTRL_CHEM_REQ
};

// Helper to get message size
inline size_t
network_msg_typ_get_size(network_msg_typ_t typ)
{
    uint8_t idx = static_cast<uint8_t>(typ);
    if (idx < ARRAY_SIZE(network_msg_typ_sizes)) {
        return network_msg_typ_sizes[idx];
    }
    return 0;
}

/*
 * Create datalink_pkt from network_msg.
 * Replaces the old X-macro loop with direct array lookups.
 */

esp_err_t
network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt)
{
    // Get protocol info from the lookup table in network_msg.h
    const network_msg_typ_info_t& info = network_msg_typ_get_info(msg->typ);
    
    // Get message size from local lookup table
    size_t data_len = network_msg_typ_get_size(msg->typ);
    
    if (data_len == 0) {
        ESP_LOGE(TAG, "unknown msg typ(%s)", network_msg_typ_str(msg->typ));
        return ESP_FAIL;
    }

    pkt->prot = info.proto;
    pkt->prot_typ = info.prot_typ;
    pkt->data_len = data_len;
    pkt->skb = skb_alloc(DATALINK_MAX_HEAD_SIZE + data_len + DATALINK_MAX_TAIL_SIZE);
    skb_reserve(pkt->skb, DATALINK_MAX_HEAD_SIZE);
    pkt->data = skb_put(pkt->skb, data_len);
    memcpy(pkt->data, msg->u.bytes, data_len);
    
    return ESP_OK;
}

} // namespace opnpool
} // namespace esphome