#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>

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

    // network_str.cpp
char const * network_ctrl_date_str(uint8_t const year, uint8_t const month, uint8_t const day);
char const * network_ctrl_time_str(uint8_t const hours, uint8_t const minutes);
char const * network_ctrl_version_str(uint8_t const major, uint8_t const minor);
const char * network_pool_mode_str(network_pool_mode_t const mode);
const char * network_pump_mode_str(network_pump_mode_t const pump_mode);
const char * network_pump_state_str(network_pump_state_t const pump_state);
char const * network_pump_program_str(uint16_t const address);
const char * network_heat_src_str(network_heat_src_t const heat_src);
char const * datalink_typ_pump_str(datalink_typ_pump_t typ);
char const * datalink_typ_ctrl_str(datalink_typ_ctrl_t typ);
char const * datalink_typ_chlor_str(datalink_typ_chlor_t typ);
const char * network_msg_typ_str(network_msg_typ_t const typ);
int network_msg_typ_nr(char const * const msg_typ_str);

}  // namespace opnpool
}  // namespace esphome