/**
 * @file network_rx.cpp
 * @brief Network layer: decode a datalink packet, to form a network message
 * 
 * @details
 * This file implements the decoding logic for the network layer of the OPNpool component.
 * It translates lower-level datalink packets (from RS-485) into higher-level network
 * messages, supporting multiple protocol types (A5/CTRL, A5/PUMP, IC/Chlorinator).
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
#include <esphome/core/log.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "network_msg.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

namespace esphome {
namespace opnpool {

static char const * const TAG = "network_rx";

    // helper to determine device_id
inline network_msg_dev_id_t
_datalink_to_network_dev_id(uint8_t const datalink_dev_id)
{
    // I only have one pump, so I have to assume that pumps are numbered sequentially starting at 0
    // I can imagine a secondary pump for solar though
    switch (datalink_dev_id) {
        case 0: return network_msg_dev_id_t::PRIMARY;
        case 1: return network_msg_dev_id_t::SECONDARY;
    }
    ESP_LOGE(TAG, "%s: unsupported datalink_dev_id %u", __FUNCTION__, datalink_dev_id);
    return network_msg_dev_id_t::PRIMARY;
}

    // helper to validate the data length of a decoded message.
static esp_err_t
_validate_data_length(network_msg_typ_t msg_typ, datalink_pkt_t const * const pkt, char const * tag, const char * typ_str)
{
    size_t expected_size;
    if (network_msg_typ_get_size(msg_typ, &expected_size) != ESP_OK) {
        ESP_LOGW(tag, "%s: failed to get expected data_len", typ_str);
        return ESP_FAIL;
    }
    if (pkt->data_len != expected_size) {
        ESP_LOGW(tag, "%s: expected data_len=%u, got data_len=%u", typ_str, expected_size, pkt->data_len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief             Decode a datalink controller packet (type A5) to form a network message
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_a5_ctrl(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_typ_ctrl_t const network_typ_ctrl = pkt->typ.ctrl;

    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs

    switch (network_typ_ctrl) {

        case datalink_typ_ctrl_t::SET_ACK:
            msg->typ = network_msg_typ_t::CTRL_SET_ACK;
            msg->u.ctrl_set_ack = *(network_msg_ctrl_set_ack_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::CIRCUIT_SET:
            msg->typ = network_msg_typ_t::CTRL_CIRCUIT_SET;
            msg->u.ctrl_circuit_set = *(network_msg_ctrl_circuit_set_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::SCHED_REQ:
            msg->typ = network_msg_typ_t::CTRL_SCHED_REQ;
            break;
        case datalink_typ_ctrl_t::SCHED_RESP:
            msg->typ = network_msg_typ_t::CTRL_SCHED_RESP;
            msg->u.ctrl_sched_resp = *(network_msg_ctrl_sched_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::STATE_BCAST:
            msg->typ = network_msg_typ_t::CTRL_STATE_BCAST;
            msg->u.ctrl_state = *(network_msg_ctrl_state_bcast_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::TIME_REQ:
            msg->typ = network_msg_typ_t::CTRL_TIME_REQ;
            break;
        case datalink_typ_ctrl_t::TIME_RESP:
            msg->typ = network_msg_typ_t::CTRL_TIME_RESP;
            msg->u.ctrl_time_resp = *(network_msg_ctrl_time_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::TIME_SET:
            msg->typ = network_msg_typ_t::CTRL_TIME_SET;
            msg->u.ctrl_time_set = *(network_msg_ctrl_time_set_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::HEAT_REQ:
            msg->typ = network_msg_typ_t::CTRL_HEAT_REQ;
            break;
        case datalink_typ_ctrl_t::HEAT_RESP:
            msg->typ = network_msg_typ_t::CTRL_HEAT_RESP;
            msg->u.ctrl_heat_resp = *(network_msg_ctrl_heat_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::HEAT_SET:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SET;
            msg->u.ctrl_heat_set = *(network_msg_ctrl_heat_set_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::LAYOUT_REQ:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_REQ;
            break;
        case datalink_typ_ctrl_t::LAYOUT_RESP:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_RESP;
            msg->u.ctrl_layout_resp = *(network_msg_ctrl_layout_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::LAYOUT_SET:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_SET;
            msg->u.ctrl_layout_set = *(network_msg_ctrl_layout_set_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::VERSION_REQ:
            msg->typ = network_msg_typ_t::CTRL_VERSION_REQ;
            msg->u.ctrl_version_req = *(network_msg_ctrl_version_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::VERSION_RESP:
            msg->typ = network_msg_typ_t::CTRL_VERSION_RESP;
            msg->u.ctrl_version_resp = *(network_msg_ctrl_version_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::VALVE_REQ:
            msg->typ = network_msg_typ_t::CTRL_VALVE_REQ;
            msg->u.ctrl_valve_req = *(network_msg_ctrl_valve_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::VALVE_RESP:
            msg->typ = network_msg_typ_t::CTRL_VALVE_RESP;
            msg->u.ctrl_valve_resp = *(network_msg_ctrl_valve_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::SOLARPUMP_REQ:
            msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_REQ;
            msg->u.ctrl_solarpump_req = *(network_msg_ctrl_solarpump_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::SOLARPUMP_RESP:
            msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_RESP;
            msg->u.ctrl_solarpump_resp = *(network_msg_ctrl_solarpump_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::DELAY_REQ:
            msg->typ = network_msg_typ_t::CTRL_DELAY_REQ;
            msg->u.ctrl_delay_req = *(network_msg_ctrl_delay_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::DELAY_RESP:
            msg->typ = network_msg_typ_t::CTRL_DELAY_RESP;
            msg->u.ctrl_delay_resp = *(network_msg_ctrl_delay_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::HEAT_SETPT_REQ:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_REQ;
            msg->u.ctrl_heat_set_req = *(network_msg_ctrl_heat_setpt_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::HEAT_SETPT_RESP:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_RESP;
            msg->u.ctrl_heat_set_resp = *(network_msg_ctrl_heat_setpt_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::SCHEDS_REQ:
            msg->typ = network_msg_typ_t::CTRL_SCHEDS_REQ;
            msg->u.ctrl_scheds_req = *(network_msg_ctrl_scheds_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::SCHEDS_RESP:
            msg->typ = network_msg_typ_t::CTRL_SCHEDS_RESP;
            msg->u.ctrl_scheds_resp = *(network_msg_ctrl_scheds_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::CIRC_NAMES_REQ:
            msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_REQ;
            msg->u.ctrl_circ_names_req = *(network_msg_ctrl_circ_names_req_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::CIRC_NAMES_RESP:
            msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_RESP;
            msg->u.ctrl_circ_names_resp = *(network_msg_ctrl_circ_names_resp_t *) pkt->data;
            break;
        case datalink_typ_ctrl_t::CHEM_REQ:
            msg->typ = network_msg_typ_t::CTRL_CHEM_REQ;
            msg->u.ctrl_chem_req = *(network_msg_ctrl_chem_req_t *) pkt->data;
            break;
        default:
            ESP_LOGW(TAG, "unknown A5_CTRL typ=%s", enum_str(network_typ_ctrl));
            return ESP_FAIL;
    }

    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(network_typ_ctrl)) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGVV(TAG, "%s: decoded A5_CTRL msg typ %s", __FUNCTION__, enum_str(msg->typ));
    return ESP_OK;
};


/**
 * @brief             Decode a datalink pump packet (type A5) to form a network message
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_a5_pump(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    bool toPump = (datalink_addr_group(pkt->dst) == datalink_addrgroup_t::PUMP);
    datalink_typ_pump_t const network_typ_pump = pkt->typ.pump;

    auto datalink_dev_id = toPump ? datalink_device_id(pkt->dst)
                                  : datalink_device_id(pkt->src);

    msg->device_id = _datalink_to_network_dev_id(datalink_dev_id);

    // 2BD: datalink_addr_id(pkt->dst, or src) will identify the specific pump device within the PUMP address group

    switch (network_typ_pump) {
        case datalink_typ_pump_t::REG:
            if (toPump) {
                msg->typ = network_msg_typ_t::PUMP_REG_SET;
                msg->u.pump_reg_set = *(network_msg_pump_reg_set_t *) pkt->data;
            } else {
                msg->typ = network_msg_typ_t::PUMP_REG_RESP;
                msg->u.pump_reg_set_resp = *(network_msg_pump_reg_resp_t *) pkt->data;
            }
            break;
        case datalink_typ_pump_t::CTRL:
            if (toPump) {
                msg->typ = network_msg_typ_t::PUMP_CTRL_SET;
            } else {
                msg->typ = network_msg_typ_t::PUMP_CTRL_RESP;
            }
            msg->u.pump_ctrl = *(network_msg_pump_ctrl_t *) pkt->data;
            break;
        case datalink_typ_pump_t::MODE:
            if (toPump) {
                msg->typ = network_msg_typ_t::PUMP_MODE_SET;
            } else {
                msg->typ = network_msg_typ_t::PUMP_MODE_RESP;
            }
            msg->u.pump_mode = *(network_msg_pump_mode_t *) pkt->data;
            break;
        case datalink_typ_pump_t::RUN:
            if (toPump) {
                msg->typ = network_msg_typ_t::PUMP_RUN_SET;
            } else {
                msg->typ = network_msg_typ_t::PUMP_RUN_RESP;
            }
            msg->u.pump_run = *(network_msg_pump_run_t *) pkt->data;
            break;
        case datalink_typ_pump_t::STATUS:
            if (toPump) {
                msg->typ = network_msg_typ_t::PUMP_STATUS_REQ;
            } else {
                msg->typ = network_msg_typ_t::PUMP_STATUS_RESP;
                msg->u.pump_status_resp = *(network_msg_pump_status_resp_t *) pkt->data;
            }
            break;
        case datalink_typ_pump_t::UNKNOWN_FF:
            ESP_LOGVV(TAG, "%s: ignoring typ (UNKNOWN_FF)", __FUNCTION__);
            return ESP_OK;
        default:
            ESP_LOGW(TAG, "unknown A5_PUMP typ=%s", enum_str(network_typ_pump));
            return ESP_FAIL;
    }

    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(network_typ_pump)) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGVV(TAG, "%s: decoded A5_PUMP msg typ %s", __FUNCTION__, enum_str(msg->typ));
    return ESP_OK;
}


/**
 * @brief             Decode a datalink chlorinator packet (type IC) to form a network message
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
static esp_err_t
_decode_msg_ic_chlor(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_typ_chlor_t const network_typ_chlor = pkt->typ.chlor;

    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs

    switch (network_typ_chlor) {
        case datalink_typ_chlor_t::PING_REQ:
            msg->typ = network_msg_typ_t::CHLOR_PING_REQ;
            msg->u.chlor_ping_req = *(network_msg_chlor_ping_req_t *) pkt->data;
            break;
        case datalink_typ_chlor_t::PING_RESP:
            msg->typ = network_msg_typ_t::CHLOR_PING_RESP;
            msg->u.chlor_ping_resp = *(network_msg_chlor_ping_resp_t *) pkt->data;
            break;
        case datalink_typ_chlor_t::NAME_RESP:
            msg->typ = network_msg_typ_t::CHLOR_NAME_RESP;
            msg->u.chlor_name_resp = *(network_msg_chlor_name_resp_t *) pkt->data;
            break;
        case datalink_typ_chlor_t::LEVEL_SET:
            msg->typ = network_msg_typ_t::CHLOR_LEVEL_SET;
            msg->u.chlor_level_set = *(network_msg_chlor_level_set_t *) pkt->data;
            break;
        case datalink_typ_chlor_t::LEVEL_RESP:
            msg->typ = network_msg_typ_t::CHLOR_LEVEL_RESP;
            msg->u.chlor_level_resp = *(network_msg_chlor_level_resp_t *) pkt->data;
            break;
        case datalink_typ_chlor_t::NAME_REQ:
            msg->typ = network_msg_typ_t::CHLOR_NAME_REQ;
            msg->u.chlor_name_req = *(network_msg_chlor_name_req_t *) pkt->data;
            break;
        default:
            ESP_LOGW(TAG, "unknown IC typ %s", enum_str(network_typ_chlor));
            return ESP_FAIL;
    }

    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(network_typ_chlor)) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGVV(TAG, "%s: decoded IC msg typ %s", __FUNCTION__, enum_str(msg->typ));
    return ESP_OK;
}


/**
 * @brief Decode a datalink packet into a network message for higher-level processing.
 *
 * This function translates a validated datalink packet (from RS-485) into a structured
 * network message, supporting multiple protocol types (A5/CTRL, A5/PUMP, IC/Chlorinator).
 * It determines the message type, populates the network message fields, and sets the
 * transmission opportunity flag if the decoded message allows for a response.
 *
 * Packets with unsupported or irrelevant destination groups are ignored. The function
 * resets the string conversion mechanism for entity names and logs decoding results for
 * debugging.
 *
 * @param pkt            Pointer to the datalink packet to decode.
 * @param msg            Pointer to the network message structure to populate.
 * @param txOpportunity  Pointer to a boolean that indicates whether the message provides a transmission opportunity.
 * @return esp_err_t     ESP_OK if the message was successfully decoded, ESP_FAIL otherwise.
 */

esp_err_t
network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity)
{
        // reset mechanism that converts various formats to string
    name_reset_idx();

        // silently ignore packets that we can't decode
    datalink_addrgroup_t const dst = datalink_addr_group(pkt->dst);
    if ((pkt->prot == datalink_prot_t::A5_CTRL && dst == datalink_addrgroup_t::X09) ||
        (pkt->prot == datalink_prot_t::IC && dst != datalink_addrgroup_t::ALL && dst != datalink_addrgroup_t::CHLOR)) {

        *txOpportunity = false;
        ESP_LOGVV(TAG, "Ignoring packet with prot %u and dst group %u", pkt->prot, dst);
        return ESP_FAIL;
    }

    esp_err_t result;

    switch (pkt->prot) {
        case datalink_prot_t::A5_CTRL:
            result = _decode_msg_a5_ctrl(pkt, msg);
            break;
        case datalink_prot_t::A5_PUMP:
            result = _decode_msg_a5_pump(pkt, msg);
            break;
        case datalink_prot_t::IC:
            result = _decode_msg_ic_chlor(pkt, msg);
            break;
        default:
            ESP_LOGW(TAG, "unknown prot %u", enum_index(pkt->prot));
            result = ESP_FAIL;
  	}
    ESP_LOGV(TAG, "Decoded pkt (prot=%s dst=%u) to %s", enum_str(pkt->prot), static_cast<uint8_t>(pkt->dst), enum_str(msg->typ));

    *txOpportunity =
        pkt->prot == datalink_prot_t::A5_CTRL &&
        datalink_addr_group(pkt->src) == datalink_addrgroup_t::CTRL &&
        datalink_addr_group(pkt->dst) == datalink_addrgroup_t::ALL;

    return result;
}

} // namespace opnpool
} // namespace esphome