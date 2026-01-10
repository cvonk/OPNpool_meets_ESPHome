/**
 * @brief OPNpool - Network layer: decode a datalink packet, to form a network message
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

/**
 * @brief             Decode a controller packet (type A5)
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_a5_ctrl(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    network_typ_ctrl_t const network_typ_ctrl = static_cast<network_typ_ctrl_t>(pkt->prot_typ);

    msg->typ = network_msg_typ_t::NONE;

    switch (network_typ_ctrl) {

        case network_typ_ctrl_t::SET_ACK:
            if (pkt->data_len == sizeof(network_msg_ctrl_set_ack_t)) {
                msg->typ = network_msg_typ_t::CTRL_SET_ACK;
                msg->u.ctrl_set_ack = *(network_msg_ctrl_set_ack_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRCUIT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_circuit_set_t)) {
                msg->typ = network_msg_typ_t::CTRL_CIRCUIT_SET;
                msg->u.ctrl_circuit_set = *(network_msg_ctrl_circuit_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHED_REQ:
            if (pkt->data_len == 0) {
                msg->typ = network_msg_typ_t::CTRL_SCHED_REQ;
            }
            break;
        case network_typ_ctrl_t::SCHED_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_sched_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_SCHED_RESP;
                msg->u.ctrl_sched_resp = *(network_msg_ctrl_sched_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::STATE_BCAST:
            if (pkt->data_len == sizeof(network_msg_ctrl_state_bcast_t)) {
                msg->typ = network_msg_typ_t::CTRL_STATE_BCAST;
                msg->u.ctrl_state = *(network_msg_ctrl_state_bcast_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::TIME_REQ:
            if (pkt->data_len == 0) {
                msg->typ = network_msg_typ_t::CTRL_TIME_REQ;
            }
            break;
        case network_typ_ctrl_t::TIME_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_time_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_TIME_RESP;
                msg->u.ctrl_time_resp = *(network_msg_ctrl_time_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::TIME_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_time_set_t)) {
                msg->typ = network_msg_typ_t::CTRL_TIME_SET;
                msg->u.ctrl_time_set = *(network_msg_ctrl_time_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_REQ:
            if (pkt->data_len == 0) {
                msg->typ = network_msg_typ_t::CTRL_HEAT_REQ;
            }
            break;
        case network_typ_ctrl_t::HEAT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_HEAT_RESP;
                msg->u.ctrl_heat_resp = *(network_msg_ctrl_heat_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_set_t)) {
                msg->typ = network_msg_typ_t::CTRL_HEAT_SET;
                msg->u.ctrl_heat_set = *(network_msg_ctrl_heat_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_REQ:
            if (pkt->data_len == 0) {
                msg->typ = network_msg_typ_t::CTRL_LAYOUT_REQ;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_layout_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_LAYOUT_RESP;
                msg->u.ctrl_layout_resp = *(network_msg_ctrl_layout_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::LAYOUT_SET:
            if (pkt->data_len == sizeof(network_msg_ctrl_layout_set_t)) {
                msg->typ = network_msg_typ_t::CTRL_LAYOUT_SET;
                msg->u.ctrl_layout_set = *(network_msg_ctrl_layout_set_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VERSION_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_version_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_VERSION_REQ;
                msg->u.ctrl_version_req = *(network_msg_ctrl_version_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VERSION_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_version_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_VERSION_RESP;
                msg->u.ctrl_version_resp = *(network_msg_ctrl_version_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VALVE_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_valve_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_VALVE_REQ;
                msg->u.ctrl_valve_req = *(network_msg_ctrl_valve_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::VALVE_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_valve_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_VALVE_RESP;
                msg->u.ctrl_valve_resp = *(network_msg_ctrl_valve_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SOLARPUMP_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_solarpump_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_REQ;
                msg->u.ctrl_solarpump_req = *(network_msg_ctrl_solarpump_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SOLARPUMP_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_solarpump_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_RESP;
                msg->u.ctrl_solarpump_resp = *(network_msg_ctrl_solarpump_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::DELAY_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_delay_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_DELAY_REQ;
                msg->u.ctrl_delay_req = *(network_msg_ctrl_delay_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::DELAY_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_delay_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_DELAY_RESP;
                msg->u.ctrl_delay_resp = *(network_msg_ctrl_delay_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SETPT_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_setpt_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_REQ;
                msg->u.ctrl_heat_set_req = *(network_msg_ctrl_heat_setpt_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::HEAT_SETPT_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_heat_setpt_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_RESP;
                msg->u.ctrl_heat_set_resp = *(network_msg_ctrl_heat_setpt_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHEDS_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_scheds_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_SCHEDS_REQ;
                msg->u.ctrl_scheds_req = *(network_msg_ctrl_scheds_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::SCHEDS_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_scheds_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_SCHEDS_RESP;
                msg->u.ctrl_scheds_resp = *(network_msg_ctrl_scheds_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRC_NAMES_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_circ_names_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_REQ;
                msg->u.ctrl_circ_names_req = *(network_msg_ctrl_circ_names_req_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CIRC_NAMES_RESP:
            if (pkt->data_len == sizeof(network_msg_ctrl_circ_names_resp_t)) {
                msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_RESP;
                msg->u.ctrl_circ_names_resp = *(network_msg_ctrl_circ_names_resp_t *) pkt->data;
            }
            break;
        case network_typ_ctrl_t::CHEM_REQ:
            if (pkt->data_len == sizeof(network_msg_ctrl_chem_req_t)) {
                msg->typ = network_msg_typ_t::CTRL_CHEM_REQ;
                msg->u.ctrl_chem_req = *(network_msg_ctrl_chem_req_t *) pkt->data;
            }
            break;
        default:
            ESP_LOGW(TAG, "unknown A5_CTRL prot_typ %s", network_typ_ctrl_str(network_typ_ctrl));
            return ESP_FAIL;
    }

    if (msg->typ == network_msg_typ_t::NONE) {
        ESP_LOGW(TAG, "failed to decode A5_CTRL prot_typ %s, wrong data_len(%u)", network_typ_ctrl_str(network_typ_ctrl), pkt->data_len);
        return ESP_FAIL;
    }
    ESP_LOGVV(TAG, "%s: decoded A5_CTRL msg typ %s", __FUNCTION__, network_msg_typ_str(msg->typ));
    return ESP_OK;
};


/**
 * @brief             Decode a pump packet (type A5)
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_a5_pump(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
	  bool toPump = (datalink_groupaddr(pkt->dst) == datalink_addrgroup_t::PUMP);

    network_typ_pump_t const network_typ_pump = static_cast<network_typ_pump_t>(pkt->prot_typ);

    msg->typ = network_msg_typ_t::NONE;

    switch (network_typ_pump) {
        case network_typ_pump_t::REG:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_reg_set_t)) {
                    msg->typ = network_msg_typ_t::PUMP_REG_SET;
                    msg->u.pump_reg_set = *(network_msg_pump_reg_set_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_reg_resp_t)) {
                    msg->typ = network_msg_typ_t::PUMP_REG_RESP;
                    msg->u.pump_reg_set_resp = *(network_msg_pump_reg_resp_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::CTRL:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_ctrl_t)) {
                    msg->typ = network_msg_typ_t::PUMP_CTRL_SET;
                    msg->u.pump_ctrl = *(network_msg_pump_ctrl_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_ctrl_t)) {
                    msg->typ = network_msg_typ_t::PUMP_CTRL_RESP;
                    msg->u.pump_ctrl = *(network_msg_pump_ctrl_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::MODE:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_mode_t)) {
                    msg->typ = network_msg_typ_t::PUMP_MODE_SET;
                    msg->u.pump_mode = *(network_msg_pump_mode_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_mode_t)) {
                    msg->typ = network_msg_typ_t::PUMP_MODE_RESP;
                    msg->u.pump_mode = *(network_msg_pump_mode_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::RUN:
            if (toPump) {
                if (pkt->data_len == sizeof(network_msg_pump_run_t)) {
                    msg->typ = network_msg_typ_t::PUMP_RUN_SET;
                    msg->u.pump_run = *(network_msg_pump_run_t *) pkt->data;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_run_t)) {
                    msg->typ = network_msg_typ_t::PUMP_RUN_RESP;
                    msg->u.pump_run = *(network_msg_pump_run_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::STATUS:
            if (toPump) {
                if (pkt->data_len == 0) {
                    msg->typ = network_msg_typ_t::PUMP_STATUS_REQ;
                }
            } else {
                if (pkt->data_len == sizeof(network_msg_pump_status_resp_t)) {
                    msg->typ = network_msg_typ_t::PUMP_STATUS_RESP;
                    msg->u.pump_status_resp = *(network_msg_pump_status_resp_t *) pkt->data;
                }
            }
            break;
        case network_typ_pump_t::UNKNOWN_FF:
            ESP_LOGVV(TAG, "%s: ignoring prot_typ (UNKNOWN_FF)", __FUNCTION__);
            return ESP_OK;
        default:
            ESP_LOGW(TAG, "unknown A5_PUMP prot_typ %s", network_typ_pump_str(network_typ_pump));
            return ESP_FAIL;
    }

    if (msg->typ == network_msg_typ_t::NONE) {
        ESP_LOGW(TAG, "failed to decode A5_PUMP prot_typ %s, wrong data_len(%u)", network_typ_pump_str(network_typ_pump), pkt->data_len);
        return ESP_FAIL;
    }
    ESP_LOGVV(TAG, "%s: decoded A5_PUMP msg typ %s", __FUNCTION__, network_msg_typ_str(msg->typ));
    return ESP_OK;
}


/**
 * @brief             Decode a chlorinator packet (type IC)
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_ic_chlor(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    network_typ_chlor_t const network_typ_chlor = static_cast<network_typ_chlor_t>(pkt->prot_typ);
  
    msg->typ = network_msg_typ_t::NONE;

    switch (network_typ_chlor) {
        case network_typ_chlor_t::PING_REQ:
            if (pkt->data_len == sizeof(network_msg_chlor_ping_req_t)) {
                msg->typ = network_msg_typ_t::CHLOR_PING_REQ;
                msg->u.chlor_ping_req = *(network_msg_chlor_ping_req_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::PING_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_ping_resp_t)) {
                msg->typ = network_msg_typ_t::CHLOR_PING_RESP;
                msg->u.chlor_ping_resp = *(network_msg_chlor_ping_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::NAME_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_name_resp_t)) {
                msg->typ = network_msg_typ_t::CHLOR_NAME_RESP;
                msg->u.chlor_name_resp = *(network_msg_chlor_name_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::LEVEL_SET:
            if (pkt->data_len == sizeof(network_msg_chlor_level_set_t)) {
                msg->typ = network_msg_typ_t::CHLOR_LEVEL_SET;
                msg->u.chlor_level_set = *(network_msg_chlor_level_set_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::LEVEL_RESP:
            if (pkt->data_len == sizeof(network_msg_chlor_level_resp_t)) {
                msg->typ = network_msg_typ_t::CHLOR_LEVEL_RESP;
                msg->u.chlor_level_resp = *(network_msg_chlor_level_resp_t *) pkt->data;
            }
            break;
        case network_typ_chlor_t::NAME_REQ:
            if (pkt->data_len == sizeof(network_msg_chlor_name_req_t)) {
                msg->typ = network_msg_typ_t::CHLOR_NAME_REQ;
                msg->u.chlor_name_req = *(network_msg_chlor_name_req_t *) pkt->data;
            }
            break;
        default:
            ESP_LOGW(TAG, "unknown IC prot_typ %s", network_typ_chlor_str(network_typ_chlor));
            return ESP_FAIL;
    } // switch

    if (msg->typ == network_msg_typ_t::NONE) {
        ESP_LOGW(TAG, "%s: failed to decode IC prot_typ (0x%02X %s), wrong data_len(%u)",
            __FUNCTION__, pkt->prot_typ, network_typ_chlor_str(network_typ_chlor),
            pkt->data_len);
        return ESP_FAIL;
    }
    ESP_LOGVV(TAG, "%s: decoded IC msg typ %s", __FUNCTION__, network_msg_typ_str(msg->typ));
    return ESP_OK;
}


/**
 * @brief                Decode a datalink packet, to form a network message
 *  
 * @param pkt            Pointer to the datalink packet to decode
 * @param msg            Pointer to the network message structure to populate
 * @param txOpportunity  Pointer to a boolean that indicates whether the message provides a transmission opportunity
 * @return esp_err_t     ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */

esp_err_t
network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity)
{
        // reset mechanism that converts various formats to string
	  name_reset_idx();

        // silently ignore packets that we can't decode
    datalink_addrgroup_t const dst = datalink_groupaddr(pkt->dst);
    if ((pkt->prot == datalink_prot_t::A5_CTRL && dst == datalink_addrgroup_t::X09) ||
        (pkt->prot == datalink_prot_t::IC && dst != datalink_addrgroup_t::ALL && dst != datalink_addrgroup_t::CHLOR)) {

        *txOpportunity = false;
        return ESP_FAIL;
    }

    esp_err_t err;

    switch (pkt->prot) {
		    case datalink_prot_t::A5_CTRL:
            err = _decode_msg_a5_ctrl(pkt, msg);
			      break;
		    case datalink_prot_t::A5_PUMP:
            err = _decode_msg_a5_pump(pkt, msg);
			      break;
		    case datalink_prot_t::IC:
            err = _decode_msg_ic_chlor(pkt, msg);
			      break;
        default:
            if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN) {
                ESP_LOGW(TAG, "unknown prot %u", pkt->prot);
            }
            err = ESP_FAIL;
  	}
    *txOpportunity =
        pkt->prot == datalink_prot_t::A5_CTRL &&
        datalink_groupaddr(pkt->src) == datalink_addrgroup_t::CTRL &&
        datalink_groupaddr(pkt->dst) == datalink_addrgroup_t::ALL;

    return err;
}

} // namespace opnpool
} // namespace esphome