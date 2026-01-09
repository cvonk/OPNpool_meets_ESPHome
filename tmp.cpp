#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

#ifndef ELEM_AT
# define ELEM_AT(a, i, v) ((uint8_t) (i) < ARRAY_SIZE(a) ? (a)[(i)] : (v))
#endif

#ifndef ELEM_POS
# define ELEM_POS(a, s) \
    do { \
      for (uint_least8_t ii = 0; ii < ARRAY_SIZE(a); ii++) { \
	    if (strcasecmp(s, a[ii]) == 0) { \
	      return ii; \
	    } \
      } \
      return -1; \
    } while(0)
#endif

    // an example
    // mapping of datalink protocols to their numeric values

#define DATALINK_PROT_MAP(XX) \
  XX(0x00, IC)      \
  XX(0x01, A5_CTRL) \
  XX(0x02, A5_PUMP) \
  XX(0xFF, NONE) \

typedef enum {
#define XX(num, name) DATALINK_PROT_##name = num,
  DATALINK_PROT_MAP(XX)
#undef XX
} datalink_prot_t;

static const char * const _datalink_prots[] = {
#define XX(num, name) #name,
  DATALINK_PROT_MAP(XX)
#undef XX
};

char const *
datalink_prot_str(datalink_prot_t const prot)
{
    return ELEM_AT(_datalink_prots, prot, hex8_str(prot));
}


    // a more involved example
    // mapping of network message types to their numeric values

#define NETWORK_MSG_TYP_MAP(XX) \
  XX( 0, NONE,             network_msg_none_t,             DATALINK_PROT_NONE,    DATALINK_PROT_NONE)              \
  XX( 1, CTRL_SET_ACK,     network_msg_ctrl_set_ack_t,     DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SET_ACK)     \
  XX( 2, CTRL_CIRCUIT_SET, network_msg_ctrl_circuit_set_t, DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_CIRCUIT_SET) \
  XX( 3, CTRL_SCHED_REQ,   network_msg_ctrl_sched_req_t,   DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SCHED_REQ)   \
  XX( 4, CTRL_SCHED_RESP,  network_msg_ctrl_sched_resp_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SCHED_RESP)  \
  XX( 5, CTRL_STATE_BCAST, network_msg_ctrl_state_bcast_t, DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_STATE_BCAST) \
  XX( 6, CTRL_TIME_REQ,    network_msg_ctrl_time_req_t,    DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_TIME_REQ)    \
  XX( 7, CTRL_TIME_RESP,   network_msg_ctrl_time_resp_t,   DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_TIME_RESP)   \
  XX( 8, CTRL_TIME_SET,    network_msg_ctrl_time_set_t,    DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_TIME_SET)    \
  XX( 9, CTRL_HEAT_REQ,    network_msg_ctrl_heat_req_t,    DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_HEAT_REQ)    \
  XX(10, CTRL_HEAT_RESP,   network_msg_ctrl_heat_resp_t,   DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_HEAT_RESP)   \
  XX(11, CTRL_HEAT_SET,    network_msg_ctrl_heat_set_t,    DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_HEAT_SET)    \
  XX(12, CTRL_LAYOUT_REQ,  network_msg_ctrl_layout_req_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_LAYOUT_REQ)  \
  XX(13, CTRL_LAYOUT_RESP, network_msg_ctrl_layout_resp_t, DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_LAYOUT_RESP) \
  XX(14, CTRL_LAYOUT_SET,  network_msg_ctrl_layout_set_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_LAYOUT_SET)  \
  XX(15, PUMP_REG_SET,     network_msg_pump_reg_set_t,     DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_REG)         \
  XX(16, PUMP_REG_RESP,    network_msg_pump_reg_resp_t,    DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_REG)         \
  XX(17, PUMP_CTRL_SET,    network_msg_pump_ctrl_t,        DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_CTRL)        \
  XX(18, PUMP_CTRL_RESP,   network_msg_pump_ctrl_t,        DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_CTRL)        \
  XX(19, PUMP_MODE_SET,    network_msg_pump_mode_t,        DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_MODE)        \
  XX(20, PUMP_MODE_RESP,   network_msg_pump_mode_t,        DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_MODE)        \
  XX(21, PUMP_RUN_SET,     network_msg_pump_run_t,         DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_RUN)         \
  XX(22, PUMP_RUN_RESP,    network_msg_pump_run_t,         DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_RUN)         \
  XX(23, PUMP_STATUS_REQ,  network_msg_pump_status_req_t,  DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_STATUS)      \
  XX(24, PUMP_STATUS_RESP, network_msg_pump_status_resp_t, DATALINK_PROT_A5_PUMP, NETWORK_TYP_PUMP_STATUS)      \
  XX(25, CHLOR_PING_REQ,   network_msg_chlor_ping_req_t,   DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_PING_REQ)   \
  XX(26, CHLOR_PING_RESP,  network_msg_chlor_ping_resp_t,  DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_PING_RESP)  \
  XX(27, CHLOR_NAME_RESP,  network_msg_chlor_name_resp_t,  DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_NAME_RESP) \
  XX(28, CHLOR_LEVEL_SET,  network_msg_chlor_level_set_t,  DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_LEVEL_SET)  \
  XX(29, CHLOR_LEVEL_RESP, network_msg_chlor_level_resp_t, DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_LEVEL_RESP) \
  XX(30, CHLOR_NAME_REQ,   network_msg_chlor_name_req_t,   DATALINK_PROT_IC,      NETWORK_TYP_CHLOR_NAME_REQ)    \
  XX(31, CTRL_VALVE_REQ,       network_msg_ctrl_valve_req_t,       DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_VALVE_REQ)       \
  XX(32, CTRL_VALVE_RESP,      network_msg_ctrl_valve_resp_t,      DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_VALVE_RESP)      \
  XX(33, CTRL_VERSION_REQ,     network_msg_ctrl_version_req_t,     DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_VERSION_REQ)     \
  XX(34, CTRL_VERSION_RESP,    network_msg_ctrl_version_resp_t,    DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_VERSION_RESP)    \
  XX(35, CTRL_SOLARPUMP_REQ,   network_msg_ctrl_solarpump_req_t,   DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SOLARPUMP_REQ)   \
  XX(36, CTRL_SOLARPUMP_RESP,  network_msg_ctrl_solarpump_resp_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SOLARPUMP_RESP)  \
  XX(37, CTRL_DELAY_REQ,       network_msg_ctrl_delay_req_t,       DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_DELAY_REQ)       \
  XX(38, CTRL_DELAY_RESP,      network_msg_ctrl_delay_resp_t,      DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_DELAY_RESP)      \
  XX(39, CTRL_HEAT_SETPT_REQ,  network_msg_ctrl_heat_setpt_req_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_HEAT_SETPT_REQ)  \
  XX(40, CTRL_HEAT_SETPT_RESP, network_msg_ctrl_heat_setpt_resp_t, DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_HEAT_SETPT_RESP) \
  XX(41, CTRL_CIRC_NAMES_REQ,  network_msg_ctrl_circ_names_req_t,  DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_CIRC_NAMES_REQ)  \
  XX(42, CTRL_CIRC_NAMES_RESP, network_msg_ctrl_circ_names_resp_t, DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_CIRC_NAMES_RESP) \
  XX(43, CTRL_SCHEDS_REQ,      network_msg_ctrl_scheds_req_t,      DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SCHEDS_REQ)      \
  XX(44, CTRL_SCHEDS_RESP,     network_msg_ctrl_scheds_resp_t,     DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_SCHEDS_RESP)     \
  XX(45, CTRL_CHEM_REQ,        network_msg_ctrl_chem_req_t,     DATALINK_PROT_A5_CTRL, NETWORK_TYP_CTRL_CHEM_REQ)
  
typedef enum {
#define XX(num, name, typ, proto, prot_typ) MSG_TYP_##name = num,
  NETWORK_MSG_TYP_MAP(XX)
#undef XX
} network_msg_typ_t;

static const char * const _network_msg_typs[] = {
#define XX(num, name, typ, proto, prot_typ) #name,
  NETWORK_MSG_TYP_MAP(XX)
#undef XX
};

const char *
network_msg_typ_str(network_msg_typ_t const typ)
{
    return ELEM_AT(_network_msg_typs, typ, hex8_str(typ));
}

int
network_msg_typ_nr(char const * const msg_typ_str)
{
    ELEM_POS(_network_msg_typs, msg_typ_str);
}
