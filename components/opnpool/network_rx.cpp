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
_get_network_dev_id(uint8_t const datalink_dev_id)
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

/**
 * @brief             Decode a A5_PUMP datalink packet to form a network message
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

    network_msg_typ_info_t const * const info = network_msg_typ_get_info(datalink_pump_typ, is_to_pump);
    if (info == nullptr) {
        ESP_LOGW(TAG, "unsupported pump_typ (%s) ", enum_str(datalink_pump_typ));
        return ESP_FAIL;
    }

    if (pkt->data_len != info->size) {
        ESP_LOGW(TAG, "%s invalid length: expected %u, got %u", enum_str(msg->typ), info->size, pkt->data_len);
        return ESP_FAIL;
    }

    auto datalink_dev_id = is_to_pump ? datalink_device_id(pkt->dst)
                                      : datalink_device_id(pkt->src);

    msg->typ       = info->network_typ;
    msg->device_id = _get_network_dev_id(datalink_dev_id);
    memcpy(msg->u.raw, pkt->data, pkt->data_len);  // honoring the union types would require a big switch() statement

    ESP_LOGVV(TAG, "%s: decoded A5_PUMP msg typ %s", __FUNCTION__, enum_str(msg->typ));
    return ESP_OK;
}


/**
 * @brief             Decode a A5_CTRL datalink packet to form a network message
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
[[nodiscard]] static esp_err_t
_decode_msg_a5_ctrl(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_ctrl_typ_t const datalink_ctrl_typ = pkt->typ.ctrl;

    network_msg_typ_info_t const * const info = network_msg_typ_get_info(datalink_ctrl_typ);
    if (info == nullptr) {
        ESP_LOGW(TAG, "unsupported ctrl_typ (%s) ", enum_str(datalink_ctrl_typ));
        return ESP_FAIL;
    }

    if (pkt->data_len != info->size) {
        ESP_LOGW(TAG, "%s invalid length: expected %u, got %u", enum_str(msg->typ), info->size, pkt->data_len);
        return ESP_FAIL;
    }

    msg->typ       = info->network_typ;
    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs
    memcpy(msg->u.raw, pkt->data, pkt->data_len);    // honoring the union types would require a big switch() statement

    ESP_LOGVV(TAG, "%s: decoded A5_CTRL msg typ %s", __FUNCTION__, enum_str(msg->typ));
    return ESP_OK;
};


/**
 * @brief             Decode a IC datalink packet to form a network message
 * 
 * @param pkt         Pointer to the datalink packet to decode
 * @param msg         Pointer to the network message structure to populate
 * @return esp_err_t  ESP_OK if the message was successfully decoded, ESP_FAIL otherwise
 */
[[nodiscard]] static esp_err_t
_decode_msg_ic_chlor(datalink_pkt_t const * const pkt, network_msg_t * const msg)
{
    datalink_chlor_typ_t const datalink_chlor_typ = pkt->typ.chlor;

    network_msg_typ_info_t const * const info = network_msg_typ_get_info(datalink_chlor_typ);
    if (info == nullptr) {
        ESP_LOGW(TAG, "unsupported chlor_typ (%s) ", enum_str(datalink_chlor_typ));
        return ESP_FAIL;
    }

    if (pkt->data_len != info->size) {
        ESP_LOGW(TAG, "%s invalid length: expected %u, got %u", enum_str(msg->typ), info->size, pkt->data_len);
        return ESP_FAIL;
    }

    msg->typ       = info->network_typ;
    msg->device_id = network_msg_dev_id_t::PRIMARY;  // only relevant for A4-PUMP msgs
    memcpy(msg->u.raw, pkt->data, pkt->data_len);    // honoring the union types would require a big switch() statement

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
        msg->typ = network_typ_t::IGNORE;
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