/**
 * @file network_rx.cpp
 * @brief Network layer: decode a datalink packet, to form a network message
 * 
 * @details
 * This file implements the decoding logic for the network layer of the OPNpool component.
 * It translates lower-level datalink packets (from RS-485) into higher-level network
 * messages, supporting multiple protocol types (A5/CTRL, A5/PUMP, IC/Chlorinator).
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

constexpr char TAG[] = "network_rx";

    // helper to determine device_id
static network_msg_dev_id_t
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
[[nodiscard]] static esp_err_t
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
[[nodiscard]] static esp_err_t
_decode_msg_a5_ctrl(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_ctrl_typ_t const datalink_ctrl_typ = pkt->typ.ctrl;

    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs

        // need to check the length first, because otherwise if pkt->data is shorter than
        // the target struct, we're reading out-of-bounds memory
    msg->typ = network_msg_typ_from_datalink(datalink_ctrl_typ);

    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(datalink_ctrl_typ)) != ESP_OK) {
        ESP_LOGW(TAG, "invalid data length for A5_CTRL msg typ=%s", enum_str(msg->typ));
        return ESP_FAIL;
    }

    switch (datalink_ctrl_typ) {

        case datalink_ctrl_typ_t::SET_ACK:
            msg->typ = network_msg_typ_t::CTRL_SET_ACK;
            msg->u.a5.ctrl_set_ack = *(network_msg_ctrl_set_ack_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::CIRCUIT_SET:
            msg->typ = network_msg_typ_t::CTRL_CIRCUIT_SET;
            msg->u.a5.ctrl_circuit_set = *(network_msg_ctrl_circuit_set_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::SCHED_REQ:
            msg->typ = network_msg_typ_t::CTRL_SCHED_REQ;
            break;
        case datalink_ctrl_typ_t::SCHED_RESP:
            msg->typ = network_msg_typ_t::CTRL_SCHED_RESP;
            msg->u.a5.ctrl_sched_resp = *(network_msg_ctrl_sched_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::STATE_BCAST:
            msg->typ = network_msg_typ_t::CTRL_STATE_BCAST;
            msg->u.a5.ctrl_state_bcast = *(network_msg_ctrl_state_bcast_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::TIME_REQ:
            msg->typ = network_msg_typ_t::CTRL_TIME_REQ;
            break;
        case datalink_ctrl_typ_t::TIME_RESP:
            msg->typ = network_msg_typ_t::CTRL_TIME_RESP;
            msg->u.a5.ctrl_time_resp = *(network_msg_ctrl_time_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::TIME_SET:
            msg->typ = network_msg_typ_t::CTRL_TIME_SET;
            msg->u.a5.ctrl_time_set = *(network_msg_ctrl_time_set_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::HEAT_REQ:
            msg->typ = network_msg_typ_t::CTRL_HEAT_REQ;
            break;
        case datalink_ctrl_typ_t::HEAT_RESP:
            msg->typ = network_msg_typ_t::CTRL_HEAT_RESP;
            msg->u.a5.ctrl_heat_resp = *(network_msg_ctrl_heat_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::HEAT_SET:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SET;
            msg->u.a5.ctrl_heat_set = *(network_msg_ctrl_heat_set_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::LAYOUT_REQ:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_REQ;
            break;
        case datalink_ctrl_typ_t::LAYOUT_RESP:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_RESP;
            msg->u.a5.ctrl_layout_resp = *(network_msg_ctrl_layout_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::LAYOUT_SET:
            msg->typ = network_msg_typ_t::CTRL_LAYOUT_SET;
            msg->u.a5.ctrl_layout_set = *(network_msg_ctrl_layout_set_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::VERSION_REQ:
            msg->typ = network_msg_typ_t::CTRL_VERSION_REQ;
            msg->u.a5.ctrl_version_req = *(network_msg_ctrl_version_req_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::VERSION_RESP:
            msg->typ = network_msg_typ_t::CTRL_VERSION_RESP;
            msg->u.a5.ctrl_version_resp = *(network_msg_ctrl_version_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::VALVE_REQ:
            msg->typ = network_msg_typ_t::CTRL_VALVE_REQ;
            break;
        case datalink_ctrl_typ_t::VALVE_RESP:
            msg->typ = network_msg_typ_t::CTRL_VALVE_RESP;
            msg->u.a5.ctrl_valve_resp = *(network_msg_ctrl_valve_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::SOLARPUMP_REQ:
            msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_REQ;
            break;
        case datalink_ctrl_typ_t::SOLARPUMP_RESP:
            msg->typ = network_msg_typ_t::CTRL_SOLARPUMP_RESP;
            msg->u.a5.ctrl_solarpump_resp = *(network_msg_ctrl_solarpump_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::DELAY_REQ:
            msg->typ = network_msg_typ_t::CTRL_DELAY_REQ;
            break;
        case datalink_ctrl_typ_t::DELAY_RESP:
            msg->typ = network_msg_typ_t::CTRL_DELAY_RESP;
            msg->u.a5.ctrl_delay_resp = *(network_msg_ctrl_delay_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::HEAT_SETPT_REQ:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_REQ;
            break;
        case datalink_ctrl_typ_t::HEAT_SETPT_RESP:
            msg->typ = network_msg_typ_t::CTRL_HEAT_SETPT_RESP;
            msg->u.a5.ctrl_heat_setpt_resp = *(network_msg_ctrl_heat_setpt_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::SCHEDS_REQ:
            msg->typ = network_msg_typ_t::CTRL_SCHEDS_REQ;
            msg->u.a5.ctrl_scheds_req = *(network_msg_ctrl_scheds_req_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::SCHEDS_RESP:
            msg->typ = network_msg_typ_t::CTRL_SCHEDS_RESP;
            msg->u.a5.ctrl_scheds_resp = *(network_msg_ctrl_scheds_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::CIRC_NAMES_REQ:
            msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_REQ;
            msg->u.a5.ctrl_circ_names_req = *(network_msg_ctrl_circ_names_req_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::CIRC_NAMES_RESP:
            msg->typ = network_msg_typ_t::CTRL_CIRC_NAMES_RESP;
            msg->u.a5.ctrl_circ_names_resp = *(network_msg_ctrl_circ_names_resp_t *) pkt->data;
            break;
        case datalink_ctrl_typ_t::CHEM_REQ:
            msg->typ = network_msg_typ_t::CTRL_CHEM_REQ;
            msg->u.a5.ctrl_chem_req = *(network_msg_ctrl_chem_req_t *) pkt->data;
            break;
        default:
            ESP_LOGW(TAG, "unknown A5_CTRL typ=%s", enum_str(datalink_ctrl_typ));
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
[[nodiscard]] static esp_err_t
_decode_msg_a5_pump(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    bool is_to_pump = (datalink_addr_group(pkt->dst) == datalink_addrgroup_t::PUMP);
    datalink_pump_typ_t const datalink_pump_typ = pkt->typ.pump;

    auto datalink_dev_id = is_to_pump ? datalink_device_id(pkt->dst)
                                      : datalink_device_id(pkt->src);

    msg->device_id = _datalink_to_network_dev_id(datalink_dev_id);

        // need to check the length first, because otherwise if pkt->data is shorter than
        // the target struct, we're reading out-of-bounds memory
    msg->typ = network_msg_typ_from_datalink(datalink_pump_typ, is_to_pump);
    
    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(datalink_pump_typ)) != ESP_OK) {
        ESP_LOGW(TAG, "invalid data length for A5_PUMP msg typ=%s", enum_str(msg->typ));
        return ESP_FAIL;
    }

    switch (datalink_pump_typ) {
        case datalink_pump_typ_t::UNKNOWN_FF:
            msg->typ = network_msg_typ_t::IGNORE;
            ESP_LOGV(TAG, "%s: ignoring typ (FF)", __FUNCTION__);
            return ESP_OK;
        case datalink_pump_typ_t::REG:
            if (is_to_pump) {
                msg->typ = network_msg_typ_t::PUMP_REG_SET;
                msg->u.a5.pump_reg_set = *(network_msg_pump_reg_set_t *) pkt->data;
            } else {
                msg->typ = network_msg_typ_t::PUMP_REG_RESP;
                msg->u.a5.pump_reg_resp = *(network_msg_pump_reg_resp_t *) pkt->data;
            }
            break;
        case datalink_pump_typ_t::CTRL:
            if (is_to_pump) {
                msg->typ = network_msg_typ_t::PUMP_CTRL_SET;
                msg->u.a5.pump_ctrl_set = *(network_msg_pump_ctrl_set_t *) pkt->data;
            } else {
                msg->typ = network_msg_typ_t::PUMP_CTRL_RESP;
                msg->u.a5.pump_ctrl_resp = *(network_msg_pump_ctrl_resp_t *) pkt->data;
            }
            break;
        case datalink_pump_typ_t::MODE:
            if (is_to_pump) {
                msg->typ = network_msg_typ_t::PUMP_MODE_SET;
                msg->u.a5.pump_mode_set = *(network_msg_pump_mode_set_t *) pkt->data;
            } else {
                msg->typ = network_msg_typ_t::PUMP_MODE_RESP;
                msg->u.a5.pump_mode_resp = *(network_msg_pump_mode_resp_t *) pkt->data;
            }
            break;
        case datalink_pump_typ_t::RUN:
            if (is_to_pump) {
                msg->typ = network_msg_typ_t::PUMP_RUN_SET;
                msg->u.a5.pump_run_set = *(network_msg_pump_run_set_t *) pkt->data;
            } else {
                msg->typ = network_msg_typ_t::PUMP_RUN_RESP;
                msg->u.a5.pump_run_resp = *(network_msg_pump_run_resp_t *) pkt->data;
            }
            break;
        case datalink_pump_typ_t::STATUS:
            if (is_to_pump) {
                msg->typ = network_msg_typ_t::PUMP_STATUS_REQ;
            } else {
                msg->typ = network_msg_typ_t::PUMP_STATUS_RESP;
                msg->u.a5.pump_status_resp = *(network_msg_pump_status_resp_t *) pkt->data;
            }
            break;
        default:
            ESP_LOGW(TAG, "unknown A5_PUMP typ=%s", enum_str(datalink_pump_typ));
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
[[nodiscard]] static esp_err_t
_decode_msg_ic_chlor(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_chlor_typ_t const datalink_chlor_typ = pkt->typ.chlor;

    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs

        // need to check the length first, because otherwise if pkt->data is shorter than
        // the target struct, we're reading out-of-bounds memory
    msg->typ = network_msg_typ_from_datalink(datalink_chlor_typ);

    if (_validate_data_length(msg->typ, pkt, TAG, enum_str(datalink_chlor_typ)) != ESP_OK) {
        ESP_LOGW(TAG, "invalid data length for IC msg typ=%s", enum_str(msg->typ));
        return ESP_FAIL;
    }

    switch (datalink_chlor_typ) {
        case datalink_chlor_typ_t::PING_REQ:
            msg->typ = network_msg_typ_t::CHLOR_PING_REQ;
            msg->u.ic.chlor_ping_req = *(network_msg_chlor_ping_req_t *) pkt->data;
            break;
        case datalink_chlor_typ_t::PING_RESP:
            msg->typ = network_msg_typ_t::CHLOR_PING_RESP;
            msg->u.ic.chlor_ping_resp = *(network_msg_chlor_ping_resp_t *) pkt->data;
            break;
        case datalink_chlor_typ_t::NAME_RESP:
            msg->typ = network_msg_typ_t::CHLOR_NAME_RESP;
            msg->u.ic.chlor_name_resp = *(network_msg_chlor_name_resp_t *) pkt->data;
            break;
        case datalink_chlor_typ_t::LEVEL_SET:
            msg->typ = network_msg_typ_t::CHLOR_LEVEL_SET;
            msg->u.ic.chlor_level_set = *(network_msg_chlor_level_set_t *) pkt->data;
            break;
        case datalink_chlor_typ_t::LEVEL_RESP:
            msg->typ = network_msg_typ_t::CHLOR_LEVEL_RESP;
            msg->u.ic.chlor_level_resp = *(network_msg_chlor_level_resp_t *) pkt->data;
            break;
        case datalink_chlor_typ_t::NAME_REQ:
            msg->typ = network_msg_typ_t::CHLOR_NAME_REQ;
            msg->u.ic.chlor_name_req = *(network_msg_chlor_name_req_t *) pkt->data;
            break;
        default:
            ESP_LOGW(TAG, "unknown IC typ %s", enum_str(datalink_chlor_typ));
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

        // silently ignore packets that we don't know how to decode
    datalink_addrgroup_t const dst = datalink_addr_group(pkt->dst);
    if ((pkt->prot == datalink_prot_t::A5_CTRL && dst == datalink_addrgroup_t::X09) ||
        (pkt->prot == datalink_prot_t::IC && dst != datalink_addrgroup_t::ALL && dst != datalink_addrgroup_t::CHLOR)) {

        *txOpportunity = false;
        msg->typ = network_msg_typ_t::IGNORE;
        ESP_LOGV(TAG, "Ignoring packet with prot %s and dst group %u", enum_str(pkt->prot), static_cast<uint8_t>(dst));
        return ESP_OK;
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