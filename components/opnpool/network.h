#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>

#include "to_str.h"
#include "enum_helpers.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {

/**
 * @brief Exported functions
 */

    // network.cpp
uint8_t network_ic_len(uint8_t const ic_typ);

    // network_rx.cpp
esp_err_t network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity);

    // network_create.cpp
esp_err_t network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt);

}  // namespace opnpool
}  // namespace esphome