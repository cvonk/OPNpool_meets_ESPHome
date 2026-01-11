#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>

#include "network_msg.h"
#include "datalink_pkt.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

namespace esphome {
namespace opnpool {

enum class network_pump_prg_addr_t : uint16_t {
    UNKN_2BF0   = 0x2BF0,
    UNKN_02BF   = 0x02BF,   
    PGM         = 0x02E4,  // program GPM
    RPM         = 0x02C4,  // program RPM
    EPRG        = 0x0321,  // select ext prog, 0x0000=P0, 0x0008=P1, 0x0010=P2, 0x0080=P3, 0x0020=P4
    ERPM0       = 0x0327,  // program ext program RPM0
    ERPM1       = 0x0328,  // program ext program RPM1
    ERPM2       = 0x0329,  // program ext program RPM2
    ERPM3       = 0x032A   // program ext program RPM3
};

/* network.c */
uint8_t network_ic_len(uint8_t const ic_typ);

/* network_rx.c */
esp_err_t network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity);

/* network_create.c */
esp_err_t network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt);

/* network_str.c */
char const * network_date_str(uint8_t const year, uint8_t const month, uint8_t const day);
char const * network_time_str(uint8_t const hours, uint8_t const minutes);
char const * network_version_str(uint8_t const major, uint8_t const minor);
const char * network_mode_str(network_mode_t const mode);
const char * network_pump_mode_str(network_pump_mode_t const pump_mode);
const char * network_pump_state_str(network_pump_state_t const pump_state);
char const * network_pump_prg_str(uint16_t const address);
const char * network_heat_src_str(network_heat_src_t const heat_src);
char const * network_typ_pump_str(network_typ_pump_t typ);
char const * network_typ_ctrl_str(network_typ_ctrl_t typ);
char const * network_typ_chlor_str(network_typ_chlor_t typ);
const char * network_msg_typ_str(network_msg_typ_t const typ);
int network_heat_src_nr(char const * const heat_src_str);
int network_circuit_nr(char const * const circuit_str);
int network_msg_typ_nr(char const * const msg_typ_str);

}  // namespace opnpool
}  // namespace esphome