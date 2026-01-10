/**
 * @brief OPNpool - Network layer: decode datalink_pkt to network_msg
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

#include <esp_system.h>
#include <esphome/core/log.h>

#include "datalink.h"
#include "datalink_pkt.h"
#include "utils.h"
#include "network.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "network_rx";

typedef struct hdr_data_hdr_copy_t {
    uint8_t dst;  // destination
    uint8_t src;  // source
    uint8_t typ;  // message type
} hdr_data_hdr_copy_t;

/*
 *
 */

static void
_decode_msg_a5_ctrl(datalink_pkt_t const * const pkt, network_msg_t * const network)
{
    network->typ = network_msg_typ_t::NONE;

    network_typ_ctrl_t network_typ_ctrl = static_cast<network_typ_ctrl_t>(pkt->prot_typ);

    switch (network_typ_ctrl) {

        case network_typ_ctrl_t::SET_ACK:
            if (pkt->data_len == sizeof(network_msg_ctrl_set_ack_t)) {
                network->typ = network_msg_typ_t::CTRL_SET_ACK;
                network->u.ctrl_set_ack = *(network_msg_ctrl_set_ack_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRCUIT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_circuit_set_t)) {
                network->typ = network_msg_typ_t::CTRL_CIRCUIT_SET;
                network->u.ctrl_circuit_set = *(network_msg_ctrl_circuit_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHED_REQ:
            if (pkt->data_len == 0) {
                network->typ = network_msg_typ_t::CTRL_SCHED_REQ;
            }
            break;
        case network_typ_ctrl_t::SCHED_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_sched_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_SCHED_RESP;
                network->u.ctrl_sched_resp = *(network_msg_ctrl_sched_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::STATE_BCAST:
            if (pkt->data_len == sizeof(network_msg_ctrl_state_bcast_t)) {
                network->typ = network_msg_typ_t::CTRL_STATE_BCAST;
                network->u.ctrl_state = *(network_msg_ctrl_state_bcast_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::TIME_REQ:
            if (pkt->data_len == 0) {
                network->typ = network_msg_typ_t::CTRL_TIME_REQ;
            }
            break;
        case network_typ_ctrl_t::TIME_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_time_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_TIME_RESP;
                network->u.ctrl_time_resp = *(network_msg_ctrl_time_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::TIME_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_time_set_t)) {
                network->typ = network_msg_typ_t::CTRL_TIME_SET;
                network->u.ctrl_time_set = *(network_msg_ctrl_time_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_REQ:
            if (pkt->data_len == 0) {
                network->typ = network_msg_typ_t::CTRL_HEAT_REQ;
            }
            break;
        case network_typ_ctrl_t::HEAT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_HEAT_RESP;
                network->u.ctrl_heat_resp = *(network_msg_ctrl_heat_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_set_t)) {
                network->typ = network_msg_typ_t::CTRL_HEAT_SET;
                network->u.ctrl_heat_set = *(network_msg_ctrl_heat_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_REQ:
            if (pkt->data_len == 0) {
                network->typ = network_msg_typ_t::CTRL_LAYOUT_REQ;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_layout_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_LAYOUT_RESP;
                network->u.ctrl_layout_resp = *(network_msg_ctrl_layout_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_layout_set_t)) {
                network->typ = network_msg_typ_t::CTRL_LAYOUT_SET;
                network->u.ctrl_layout_set = *(network_msg_ctrl_layout_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VERSION_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_version_req_t)) {
                network->typ = network_msg_typ_t::CTRL_VERSION_REQ;
                network->u.ctrl_version_req = *(network_msg_ctrl_version_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VERSION_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_version_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_VERSION_RESP;
                network->u.ctrl_version_resp = *(network_msg_ctrl_version_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VALVE_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_valve_req_t)) {
                network->typ = network_msg_typ_t::CTRL_VALVE_REQ;
                network->u.ctrl_valve_req = *(network_msg_ctrl_valve_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VALVE_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_valve_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_VALVE_RESP;
                network->u.ctrl_valve_resp = *(network_msg_ctrl_valve_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SOLARPUMP_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_solarpump_req_t)) {
                network->typ = network_msg_typ_t::CTRL_SOLARPUMP_REQ;
                network->u.ctrl_solarpump_req = *(network_msg_ctrl_solarpump_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SOLARPUMP_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_solarpump_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_SOLARPUMP_RESP;
                network->u.ctrl_solarpump_resp = *(network_msg_ctrl_solarpump_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::DELAY_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_delay_req_t)) {
                network->typ = network_msg_typ_t::CTRL_DELAY_REQ;
                network->u.ctrl_delay_req = *(network_msg_ctrl_delay_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::DELAY_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_delay_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_DELAY_RESP;
                network->u.ctrl_delay_resp = *(network_msg_ctrl_delay_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SETPT_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_setpt_req_t)) {
                network->typ = network_msg_typ_t::CTRL_HEAT_SETPT_REQ;
                network->u.ctrl_heat_set_req = *(network_msg_ctrl_heat_setpt_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SETPT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_setpt_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_HEAT_SETPT_RESP;
                network->u.ctrl_heat_set_resp = *(network_msg_ctrl_heat_setpt_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHEDS_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_scheds_req_t)) {
                network->typ = network_msg_typ_t::CTRL_SCHEDS_REQ;
                network->u.ctrl_scheds_req = *(network_msg_ctrl_scheds_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHEDS_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_scheds_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_SCHEDS_RESP;
                network->u.ctrl_scheds_resp = *(network_msg_ctrl_scheds_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRC_NAMES_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_circ_names_req_t)) {
                network->typ = network_msg_typ_t::CTRL_CIRC_NAMES_REQ;
                network->u.ctrl_circ_names_req = *(network_msg_ctrl_circ_names_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRC_NAMES_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_circ_names_resp_t)) {
                network->typ = network_msg_typ_t::CTRL_CIRC_NAMES_RESP;
                network->u.ctrl_circ_names_resp = *(network_msg_ctrl_circ_names_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CHEM_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_chem_req_t)) {
                network->typ = network_msg_typ_t::CTRL_CHEM_REQ;
                network->u.ctrl_chem_req = *(network_msg_ctrl_chem_req_t *) pkt->data;
            }
            break;
        default:
            ESP_LOGW(TAG, "unknown A5_CTRL pkt->prot_typ (0x%02X)", pkt->prot_typ);
            break;
    }
};

/*
 *
 */

static void
_decode_msg_a5_pump(datalink_pkt_t const * const pkt, network_msg_t * const network)
{
    network->typ = network_msg_typ_t::NONE;

	  bool toPump = (datalink_groupaddr(pkt->dst) == datalink_addrgroup_t::PUMP);

    network_typ_pump_t network_typ_pump = static_cast<network_typ_pump_t>(pkt->prot_typ);

    switch (network_typ_pump) {
        case network_typ_pump_t::REG:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_reg_set_t)) {
                    network->typ = network_msg_typ_t::PUMP_REG_SET;
                    network->u.pump_reg_set = *(network_msg_pump_reg_set_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_reg_resp_t)) {
                    network->typ = network_msg_typ_t::PUMP_REG_RESP;
                    network->u.pump_reg_set_resp = *(network_msg_pump_reg_resp_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::CTRL:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_ctrl_t)) {
                    network->typ = network_msg_typ_t::PUMP_CTRL_SET;
                    network->u.pump_ctrl = *(network_msg_pump_ctrl_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_ctrl_t)) {
                    network->typ = network_msg_typ_t::PUMP_CTRL_RESP;
                    network->u.pump_ctrl = *(network_msg_pump_ctrl_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::MODE:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_mode_t)) {
                    network->typ = network_msg_typ_t::PUMP_MODE_SET;
                    network->u.pump_mode = *(network_msg_pump_mode_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_mode_t)) {
                    network->typ = network_msg_typ_t::PUMP_MODE_RESP;
                    network->u.pump_mode = *(network_msg_pump_mode_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::RUN:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_run_t)) {
                    network->typ = network_msg_typ_t::PUMP_RUN_SET;
                    network->u.pump_run = *(network_msg_pump_run_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_run_t)) {
                    network->typ = network_msg_typ_t::PUMP_RUN_RESP;
                    network->u.pump_run = *(network_msg_pump_run_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::STATUS:
            if (toPump) {
                if (pkt->data_len == 0) {
                    network->typ = network_msg_typ_t::PUMP_STATUS_REQ;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_status_resp_t)) {
                    network->typ = network_msg_typ_t::PUMP_STATUS_RESP;
                    network->u.pump_status_resp = *(network_msg_pump_status_resp_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::FF:
            // silently ignore
            break;
        default:
            if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN) {
                ESP_LOGW(TAG, "unknown A5 pump typ %u", pkt->prot_typ);
            }
            break;
    }
}

/*
 *
 */

static void
_decode_msg_ic_chlor(datalink_pkt_t const * const pkt, network_msg_t * const network)
{
    network->typ = network_msg_typ_t::NONE;

    network_typ_chlor_t network_typ_chlor = static_cast<network_typ_chlor_t>(pkt->prot_typ);
    
    switch (network_typ_chlor) {
        case network_typ_chlor_t::PING_REQ:
            if (pkt->data_len == sizeof(network_msg_chlor_ping_req_t)) {
                network->typ = network_msg_typ_t::CHLOR_PING_REQ;
                network->u.chlor_ping_req = *(network_msg_chlor_ping_req_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::PING_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_ping_resp_t)) {
                network->typ = network_msg_typ_t::CHLOR_PING_RESP;
                network->u.chlor_ping = *(network_msg_chlor_ping_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::NAME_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_name_resp_t)) {
                network->typ = network_msg_typ_t::CHLOR_NAME_RESP;
                network->u.chlor_name_resp = *(network_msg_chlor_name_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::LEVEL_SET:
            if (pkt->data_len == sizeof(network_msg_chlor_level_set_t)) {
                network->typ = network_msg_typ_t::CHLOR_LEVEL_SET;
                network->u.chlor_level_set = *(network_msg_chlor_level_set_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::LEVEL_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_level_resp_t)) {
                network->typ = network_msg_typ_t::CHLOR_LEVEL_RESP;
                network->u.chlor_level_resp = *(network_msg_chlor_level_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::NAME_REQ:
            if (pkt->data_len == sizeof(network_msg_chlor_name_req_t)) {
                network->typ = network_msg_typ_t::CHLOR_NAME_REQ;
                network->u.chlor_name_req = *(network_msg_chlor_name_req_t *) pkt->data;
            }
            break;
        default:
            if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN) {
                ESP_LOGW(TAG, "unknown IC typ %u", pkt->prot_typ);
            }
            break;
    }
}

/*
 * 
 */

esp_err_t
network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity)
{
    // reset mechanism that converts various formats to string
	name_reset_idx();

#if 1
    // silently ignore packets that we can't decode
    datalink_addrgroup_t const dst = datalink_groupaddr(pkt->dst);
    if ((pkt->prot == datalink_prot_t::A5_CTRL && dst == datalink_addrgroup_t::X09) ||
        (pkt->prot == datalink_prot_t::IC && dst != datalink_addrgroup_t::ALL && dst != datalink_addrgroup_t::CHLOR)) {
        return ESP_FAIL;
    }
#endif
	  switch (pkt->prot) {
		    case datalink_prot_t::A5_CTRL:
            _decode_msg_a5_ctrl(pkt, msg);
			      break;
		    case datalink_prot_t::A5_PUMP:
            _decode_msg_a5_pump(pkt, msg);
			      break;
		    case datalink_prot_t::IC:
            _decode_msg_ic_chlor(pkt, msg);
			      break;
        default:
            if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN) {
                ESP_LOGW(TAG, "unknown prot %u", pkt->prot);
            }
  	}
    *txOpportunity =
        pkt->prot == datalink_prot_t::A5_CTRL &&
        datalink_groupaddr(pkt->src) == datalink_addrgroup_t::CTRL &&
        datalink_groupaddr(pkt->dst) == datalink_addrgroup_t::ALL;

    return msg->typ == network_msg_typ_t::NONE ? ESP_FAIL : ESP_OK;
}

} // namespace opnpool
} // namespace esphome