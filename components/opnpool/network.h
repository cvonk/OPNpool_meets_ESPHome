#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct datalink_pkt_t;
struct network_msg_t;

    // network_rx.cpp
esp_err_t network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity);

    // network_create.cpp
esp_err_t network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt);

}  // namespace opnpool
}  // namespace esphome