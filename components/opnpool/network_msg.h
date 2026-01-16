/**
 * @file network_msg.h
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Defines the network message structure for the OPNpool component.
 * 
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * 
 * @details
 * This header defines the network message structures, enums, and helper functions for the OPNpool component.
 * It specifies protocol-level message types, controller operation modes, circuits, pump modes, heat sources,
 * and their string conversion utilities. The file provides packed C-style structs for all supported protocol
 * messages exchanged between the ESPHome component and pool equipment, as well as lookup tables and functions
 * for message type metadata and size validation.
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

#pragma once
#ifndef __cplusplus
# error "This header requires C++ compilation"
#endif

#include <esp_system.h>
#include <cstddef>

#include "to_str.h"
#include "enum_helpers.h"
#include "datalink_pkt.h"
#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"

namespace esphome {
namespace opnpool {


/**
 * @brief Enumerates the operation modes of the pool controller.
 *
 * @details
 * Represents the various modes in which the pool controller can operate, such as service mode,
 * temperature increase, freeze protection, and timeout. Used for status reporting and control logic.
 */

enum class network_pool_mode_t : uint8_t {
    SERVICE     = 0,
    UNKNOWN_01  = 1,
    TEMP_INC    = 2,
    FREEZE_PROT = 3,
    TIMEOUT     = 4
};

#ifdef __INTELLISENSE__
#define NETWORK_POOL_MODE_COUNT (5)   // IntelliSense doesn't evaluate constexpr functions, use temporary constant
#else
#define NETWORK_POOL_MODE_COUNT (enum_count<network_pool_mode_t>())
#endif

/**
 * @brief Enumerates the pool controller circuits and features.
 *
 * @details
 * Represents the various circuits and features (such as SPA, AUX, POOL, and FEATURE channels)
 * that can be controlled by the pool controller.
 */

enum class network_pool_circuit_t : uint8_t {
    SPA      = 0,
    AUX1     = 1,
    AUX2     = 2,
    AUX3     = 3,
    FEATURE1 = 4,
    POOL     = 5,
    FEATURE2 = 6,
    FEATURE3 = 7,
    FEATURE4 = 8
};
  
#ifdef __INTELLISENSE__
#define NETWORK_POOL_CIRCUIT_COUNT (9)   // IntelliSense doesn't evaluate constexpr functions, use temporary constant
#else
#define NETWORK_POOL_CIRCUIT_COUNT (enum_count<network_pool_circuit_t>())
#endif


/**
 * @brief Enumerates the operation modes of the pool pump.
 *
 * @details
 * Represents the various modes in which the pool pump can operate, such as filter, 
 * manual, backwash and features. Used for status reporting.
 */
enum class network_pump_mode_t : uint8_t {
    FILTER = 0,
    MAN    = 1,
    BKWASH = 2,
    X03    = 3,
    X04    = 4,
    X05    = 5,
    FT1    = 6,
    X07    = 7,
    X08    = 8,
    EP1    = 9,
    EP2    = 10,
    EP3    = 11,
    EP4    = 12
};

inline const char *
enum_str(network_pump_mode_t const mode)
{
    auto name = magic_enum::enum_name(mode);
    if (!name.empty()) {
        return name.data();
    }
        // fallback
    static char buf[3];  // safe çause FreeRTOS uses coorporative scheduling
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(mode));
    return buf;
}


/**
 * @brief Enumerates the states of the pool pump.
 *
 * @details
 * Represents the various states in which the pool pump can be, such as OK,
 * priming, running, and system priming.
 */

enum class network_pump_state_t : uint8_t {
    OK         = 0,
    PRIMING    = 1,
    RUNNING    = 2,
    X03        = 3,
    SYSPRIMING = 4
};

inline const char *
enum_str(network_pump_state_t const pump_state)
{
    auto name = magic_enum::enum_name(pump_state);
    if (!name.empty()) {
        return name.data();
    }
        // fallback
    static char buf[3];  // safe çause FreeRTOS uses coorporative scheduling
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(pump_state));
    return buf;
}

 /**
  * @brief Enumerates the heat sources for the pool heating system.
  *
  * @details
  * Represents the various heat sources that can be used for heating the pool,
  * such as none, heater, solar preference, and solar.
  */

enum class network_heat_src_t : uint8_t {
    NONE = 0,
    HEATER = 1,
    SOLAR_PREF = 2,
    SOLAR = 3
};

inline const char *
enum_str(network_heat_src_t const heat_src)
{
    auto name = magic_enum::enum_name(heat_src);
    if (!name.empty()) {
        return name.data();
    }
        // fallback
    static char buf[3];  // safe çause FreeRTOS uses coorporative scheduling
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(heat_src));
    return buf;
}


/**
 * @brief Defines the structures and unions for A5-controller messages.
 *
 * @details
 * This section provides packed C-style structs for each A5-pump style
 * protocol message exchanged between the pool controller and OPNpool.
 */

struct network_msg_ctrl_set_ack_t {
    uint8_t typ;  // datalink_typ_ctrl_t::*  type that it is ACK'ing
} PACK8;

struct network_msg_ctrl_circuit_set_t {
    uint8_t circuit;  // 1-based
    uint8_t value;
} PACK8;

struct network_msg_ctrl_sched_req_t {
    // note: sizeof(network_msg_ctrl_sched_req_t) == 1, not 0
};

struct network_msg_ctrl_sched_resp_sub_t {
    uint8_t circuit;            // 0  (0 = schedule not active)
    uint8_t UNKNOWN_1;          // 1
    uint8_t prgStartHi;         // 2 [min]
    uint8_t prgStartLo;         // 3 [min]
    uint8_t prgStopHi;          // 4 [min]
    uint8_t prgStopLo;          // 5 [min]
} PACK8;

constexpr size_t NETWORK_MSG_CTRL_SCHED_COUNT = 2;

struct network_msg_ctrl_sched_resp_t {
    uint8_t                           UNKNOWN_0to3[4];  // 0,1,2,3
    network_msg_ctrl_sched_resp_sub_t scheds[NETWORK_MSG_CTRL_SCHED_COUNT]; // 4,5,6,7,8,9, 10,11,12,13,14,15
} PACK8;

struct network_msg_ctrl_state_bcast_t {
    uint8_t hour;               // 0
    uint8_t minute;             // 1
    uint8_t activeLo;           // 2
    uint8_t activeHi;           // 3
    uint8_t UNKNOWN_4to6[3];    // 4..6 more `active` circuits on fancy controllers
    uint8_t UNKNOWN_7to8[2];    // 7..8
    uint8_t modes;              // 9
    uint8_t heatStatus;         // 10
    uint8_t UNKNOWN_11;         // 11
    uint8_t delay;              // 12
    uint8_t UNKNOWN_13;         // 13
    uint8_t poolTemp;           // 14 water sensor 1
    uint8_t spaTemp;            // 15 water sensor 2
    uint8_t UNKNOWN_16;         // 16 unknown          (was major)
    uint8_t waterTemp;          // 17 solar sensor 1   (was minor)
    uint8_t airTemp;            // 18 air sensor
    uint8_t waterTemp2;         // 19 solar sensor 2
    uint8_t UNKNOWN_20tp21[2];  // 20..21 more water sensors?
    uint8_t heatSrc;            // 22 
    uint8_t UNKNOWN_23to28[6];  // 23..28
} PACK8;

// just a guess
using network_msg_ctrl_state_set_t = network_msg_ctrl_state_bcast_t;

struct network_msg_ctrl_time_req_t {
    // note: sizeof(network_msg_ctrl_time_req_t) == 1, not 0
};

struct network_msg_ctrl_time_t {
    uint8_t hour;            // 0
    uint8_t minute;          // 1
    uint8_t UNKNOWN_2;       // 2 (DST adjust?)
    uint8_t day;             // 3
    uint8_t month;           // 4
    uint8_t year;            // 5
    uint8_t clkSpeed;        // 6
    uint8_t daylightSavings; // 7 (1=auto, 0=manual)
} PACK8;

using network_msg_ctrl_time_set_t = network_msg_ctrl_time_t;
using network_msg_ctrl_time_resp_t = network_msg_ctrl_time_t;

struct network_msg_ctrl_version_req_t {
    // note: sizeof(network_msg_ctrl_version_req_t) == 1, not 0
    // uint8_t reqId;
};

struct network_msg_ctrl_version_resp_t {
    uint8_t reqId;       // 0
    uint8_t major;       // 1    0x02  -> version 2.080
    uint8_t minor;       // 2    0x50
    uint8_t UNK3to4[2];  // 3,4
    uint8_t bootMajor;   // 5
    uint8_t bootMinor;   // 6   
    uint8_t U07to16[10]; // 7,8,9,10,11, 12,13,14,15,16
};

struct network_msg_ctrl_valve_req_t {
    // note: sizeof(network_msg_ctrl_valve_req_t) == 1, not 0
};

struct network_msg_ctrl_valve_resp_t {
    uint8_t UNKNOWN[24]; // 03 00 00 00 00 FF FF 01 02 03 04 01 48 00 00 00 03 00 00 00 04 00 00 00
};

struct network_msg_ctrl_solarpump_req_t {
    // note: sizeof(network_msg_ctrl_solarpump_req_t) == 1, not 0
};

struct network_msg_ctrl_solarpump_resp_t {
    uint8_t UNKNOWN[3];  // 05 00 00
};

struct network_msg_ctrl_delay_req_t {
    // note: sizeof(network_msg_ctrl_delay_req_t) == 1, not 0
};

struct network_msg_ctrl_delay_resp_t {
    uint8_t UNKNOWN[2];  // 10 00
};

struct network_msg_ctrl_heat_setpt_req_t {
    // note: sizeof(network_msg_ctrl_heat_setpt_req_t) == 1, not 0
};

struct network_msg_ctrl_heat_setpt_resp_t {
    uint8_t UNKNOWN[10];  // 00 00 00 00 00 00 00 00 00 00 
};

struct network_msg_ctrl_circ_names_req_t {
    uint8_t reqId;  // 0x01
};

struct network_msg_ctrl_circ_names_resp_t {
    uint8_t reqId;       // req 0x01 -> resp 01 01 48 00 00
    uint8_t UNKNOWN[5];  // req 0x02 -> resp 02 00 03 00 00
};

struct network_msg_ctrl_chem_req_t {
    uint8_t UNKNOWN;  // 0xD2
};;

struct network_msg_ctrl_scheds_req_t {
    uint8_t schedId;  // 0x01 (1 - 12)
};

// With POOL 1/1 from 08:00 to 10:00, and
//      SPA  1/1 from 11:00 to 12:00
// sending [0] => no response
// sending [1] => 01 01 08 00 00 00 3F
// sending [2] => 02 01 0C 30 15 14 3F
// sending [3] => 03 00 2E 38 08 25 3F
//
// With no POOL, no SPA, no ..
// sending [1] => 01 00 00 00 00 00 3F
// sending [2] => 02 01 0C 30 15 14 3F
// sending [3] => 03 00 2E 38 08 25 3F
//
// With only POOL 1/1 from 08:00 to 10:00
// sending [1] => 01 06 00 00 00 00 3F
// sending [2] => 02 01 0C 30 15 14 3F
// sending [3] => 03 00 2E 38 08 25 3F

struct network_msg_ctrl_scheds_resp_t {
    uint8_t schedId;  // 0 
    uint8_t circuit;  // 1
    uint8_t startHr;  // 2
    uint8_t startMin; // 3
    uint8_t stopHr;   // 4
    uint8_t stopMin;  // 5
    uint8_t dayOfWk;  // 6 bitmask Mon (0x01), Tue (0x02), Wed (0x04), Thu(0x08), Fri (0x10), Sat (0x20), Sun(0x40)
};

struct network_msg_ctrl_heat_req_t {
    // note: sizeof(network_msg_ctrl_heat_req_t) == 1, not 0
};

struct network_msg_ctrl_heat_resp_t {
    uint8_t poolTemp;      // 0
    uint8_t spaTemp;       // 1
    uint8_t airTemp;       // 2
    uint8_t poolSetpoint;  // 3
    uint8_t spaSetpoint;   // 4
    uint8_t heatSrc;       // 5
    uint8_t UNKNOWN_6;     // 6
    uint8_t UNKNOWN_7;     // 7
    uint8_t UNKNOWN_8;     // 8
    uint8_t UNKNOWN_9;     // 9
    uint8_t UNKNOWN_10;    // 10
    uint8_t UNKNOWN_11;    // 11
    uint8_t UNKNOWN_12;    // 12
} PACK8;

struct network_msg_ctrl_heat_set_t {
    uint8_t poolSetpoint;  // 0
    uint8_t spaSetpoint;   // 1
    uint8_t heatSrc;       // 2
    uint8_t UNKNOWN_3;     // 3
} PACK8;

struct network_msg_ctrl_layout_req_t {
    // be aware: sizeof(network_msg_ctrl_layout_req_t) == 1, not 0
};

struct network_msg_ctrl_layout_resp_t {
    uint8_t circuit[4];  // circuits assigned to each of the 4 buttons on the remote
} PACK8;

using network_msg_ctrl_layout_set_t = network_msg_ctrl_layout_resp_t;


/**
 * @brief
 * Defines the structures and unions for A5-pump messages.
 *
 * @details
 * This section provides packed C-style structs for each A5-pump style
 * protocol message exchanged between the pool controller and its pump.
 */

struct network_msg_pump_reg_set_t {
    uint8_t addressHi;   // 0
    uint8_t addressLo;   // 1
    uint8_t valueHi;     // 2
    uint8_t valueLo;     // 3
} PACK8;

struct network_msg_pump_reg_resp_t {
    uint8_t valueHi;     // 0
    uint8_t valueLo;     // 1
} PACK8;

struct network_msg_pump_ctrl_t {
    uint8_t ctrl;        // 0
} PACK8;

struct network_msg_pump_mode_t {
    uint8_t mode;        // 0
} PACK8;

struct network_msg_pump_run_t {
    uint8_t running;     // 0
} PACK8;

struct network_msg_pump_status_req_t {
    // note: sizeof(network_msg_pump_status_req_t) == 1, not 0
};

enum class network_pump_program_addr_t : uint16_t {
    UNKNOWN_2BF0 = 0x2BF0,
    UNKNOWN_02BF = 0x02BF,   
    PGM          = 0x02E4,  // program GPM
    RPM          = 0x02C4,  // program RPM
    EPRG         = 0x0321,  // select ext prog, 0x0000=P0, 0x0008=P1, 0x0010=P2, 0x0080=P3, 0x0020=P4
    ERPM0        = 0x0327,  // program ext program RPM0
    ERPM1        = 0x0328,  // program ext program RPM1
    ERPM2        = 0x0329,  // program ext program RPM2
    ERPM3        = 0x032A   // program ext program RPM3
};

    // can't use magic enum, because  enum the values that are not contiguous, and outside the range that magic_enum expects
inline const char * 
network_pump_program_addr_str(network_pump_program_addr_t addr)
{
    switch (addr) {
        case network_pump_program_addr_t::UNKNOWN_2BF0: return "2BF0";
        case network_pump_program_addr_t::UNKNOWN_02BF: return "02BF";
        case network_pump_program_addr_t::PGM:          return "pgm";
        case network_pump_program_addr_t::RPM:          return "rpm";
        case network_pump_program_addr_t::EPRG:         return "eprg";
        case network_pump_program_addr_t::ERPM0:        return "erpm0";
        case network_pump_program_addr_t::ERPM1:        return "erpm1";
        case network_pump_program_addr_t::ERPM2:        return "erpm2";
        case network_pump_program_addr_t::ERPM3:        return "erpm3";
        default: return uint16_str(static_cast<uint16_t>(addr));
    }
}

struct network_msg_pump_status_resp_t {
    uint8_t running;      // 0
    uint8_t mode;         // 1
    uint8_t state;        // 2
    uint8_t powerHi;      // 3
    uint8_t powerLo;      // 4 [Watt]
    uint8_t speedHi;      // 5
    uint8_t speedLo;      // 6 [rpm]
    uint8_t flow;         // 7 [G/min]
    uint8_t level;        // 8 [%]
    uint8_t UNKNOWN_9;    // 9
    uint8_t error;        // 10
    uint8_t remainingHr;  // 11
    uint8_t remainingMin; // 12
    uint8_t clockHr;      // 13
    uint8_t clockMin;     // 14
} PACK8;

/**
 * @brief Defines the structures and unions for IC messages.
 *
 * @details
 * This section provides packed C-style structs for each IC style
 * protocol message exchanged between the pool controller and the
 * IntelliChlor chlorinator.
 */

struct network_msg_chlor_ping_req_t {
    uint8_t UNKNOWN_0;
} PACK8;

struct network_msg_chlor_ping_resp_t {
    uint8_t UNKNOWN_0;
    uint8_t UNKNOWN_1;
} PACK8;

using network_msg_chlor_name_str_t = char[16];

struct network_msg_chlor_name_req_t {
    uint8_t UNKNOWN;  // Sending 0x00 or 0x02 gets a response
} PACK8;

struct network_msg_chlor_name_resp_t {
    uint8_t                      salt;  // ppm/50
    network_msg_chlor_name_str_t name;
} PACK8;

struct network_msg_chlor_level_set_t {
    uint8_t  level;
} PACK8;

struct network_msg_chlor_level_resp_t {
    uint8_t  salt;   // ppm/50
    uint8_t  error;  // error bits: low flow (0x01), low salt (0x02), high salt (0x04), clean cell (0x10), cold (0x40), OK (0x80)
} PACK8;


/**
 * @brief Defines unions for grouping protocol message data for A5 and IC network messages.
 *
 * @details
 * These unions encapsulate all supported message types for the A5 (controller/pump) and IC
 * (chlorinator) protocols, allowing flexible access to protocol-specific message structures.
 * The top-level union `network_msg_data_t` enables generic handling of any protocol message
 * within the OPNpool system, simplifying encoding, decoding, and processing of network
 * messages.
 */

union network_msg_data_a5_t {
    network_msg_pump_reg_set_t         pump_reg_set;
    network_msg_pump_reg_resp_t        pump_reg_set_resp;
    network_msg_pump_ctrl_t            pump_ctrl;
    network_msg_pump_mode_t            pump_mode;
    network_msg_pump_run_t             pump_run;
    network_msg_pump_status_resp_t     pump_status;
    network_msg_ctrl_set_ack_t         ctrl_set_ack;
    network_msg_ctrl_circuit_set_t     ctrl_circuit_set;
    network_msg_ctrl_sched_resp_t      ctrl_sched;
    network_msg_ctrl_state_bcast_t     ctrl_state;
    network_msg_ctrl_state_set_t       ctrl_state_set;
    network_msg_ctrl_time_resp_t       ctrl_time_resp;
    network_msg_ctrl_heat_resp_t       ctrl_heat_resp;
    network_msg_ctrl_heat_set_t        ctrl_heat_set;
    network_msg_ctrl_layout_resp_t     ctrl_layout_resp;
    network_msg_ctrl_layout_set_t      ctrl_layout_set;    
    network_msg_ctrl_version_req_t     ctrl_version_req;
    network_msg_ctrl_version_resp_t    ctrl_version_resp;
    network_msg_ctrl_valve_req_t       ctrl_valve_req;
    network_msg_ctrl_valve_resp_t      ctrl_valve_resp;
    network_msg_ctrl_solarpump_req_t   ctrl_solarpump_req;
    network_msg_ctrl_solarpump_resp_t  ctrl_solarpump_resp;
    network_msg_ctrl_delay_req_t       ctrl_delay_req;
    network_msg_ctrl_delay_resp_t      ctrl_delay_resp;
    network_msg_ctrl_heat_setpt_req_t  ctrl_heat_setpt_req;
    network_msg_ctrl_heat_setpt_resp_t ctrl_heat_setpt_resp;
    network_msg_ctrl_circ_names_req_t  ctrl_circ_names_req;
    network_msg_ctrl_circ_names_resp_t ctrl_circ_names_resp;
    network_msg_ctrl_chem_req_t        ctrl_chem_req;
    network_msg_ctrl_scheds_req_t      ctrl_scheds_req;
    network_msg_ctrl_scheds_resp_t     ctrl_scheds_resp;
} PACK8;

union network_msg_data_ic_t {
    network_msg_chlor_ping_req_t    chlor_ping_req;
    network_msg_chlor_ping_resp_t   chlor_ping_resp;
    network_msg_chlor_name_req_t    chlor_name_req;
    network_msg_chlor_name_resp_t   chlor_name_resp;
    network_msg_chlor_level_set_t   chlor_level_set;
    network_msg_chlor_level_resp_t  chlor_level_resp;
} PACK8;

union network_msg_data_t {
    network_msg_data_a5_t a5;
    network_msg_data_ic_t ic;
} PACK8;

const size_t NETWORK_DATA_MAX_SIZE = sizeof(network_msg_data_t);


/**
 * @brief
 * Enumerates all supported network message types for OPNpool.
 *
 * @details
 * Each value represents a specific protocol message exchanged between the ESPHome component and pool equipment,
 * including controller, pump, and chlorinator messages. Used for message dispatch, parsing, and type-safe handling.
 */
enum class network_msg_typ_t : uint8_t {  // MUST MATCH network_msg_typ_info[] and network_msg_typ_sizes[]
    CTRL_SET_ACK = 0,
    CTRL_CIRCUIT_SET = 1,
    CTRL_SCHED_REQ = 2,
    CTRL_SCHED_RESP = 3,
    CTRL_STATE_BCAST = 4,
    CTRL_TIME_REQ = 5,
    CTRL_TIME_RESP = 6,
    CTRL_TIME_SET = 7,
    CTRL_HEAT_REQ = 8,
    CTRL_HEAT_RESP = 9,
    CTRL_HEAT_SET = 10,
    CTRL_LAYOUT_REQ = 11,
    CTRL_LAYOUT_RESP = 12,
    CTRL_LAYOUT_SET = 13,
    PUMP_REG_SET = 14,
    PUMP_REG_RESP = 15,
    PUMP_CTRL_SET = 16,
    PUMP_CTRL_RESP = 17,
    PUMP_MODE_SET = 18,
    PUMP_MODE_RESP = 19,
    PUMP_RUN_SET = 20,
    PUMP_RUN_RESP = 21,
    PUMP_STATUS_REQ = 22,
    PUMP_STATUS_RESP = 23,
    CHLOR_PING_REQ = 24,
    CHLOR_PING_RESP = 25,
    CHLOR_NAME_RESP = 26,
    CHLOR_LEVEL_SET = 27,
    CHLOR_LEVEL_RESP = 28,
    CHLOR_NAME_REQ = 29,
    CTRL_VALVE_REQ = 30,
    CTRL_VALVE_RESP = 31,
    CTRL_VERSION_REQ = 32,
    CTRL_VERSION_RESP = 33,
    CTRL_SOLARPUMP_REQ = 34,
    CTRL_SOLARPUMP_RESP = 35,
    CTRL_DELAY_REQ = 36,
    CTRL_DELAY_RESP = 37,
    CTRL_HEAT_SETPT_REQ = 38,
    CTRL_HEAT_SETPT_RESP = 39,
    CTRL_CIRC_NAMES_REQ = 40,
    CTRL_CIRC_NAMES_RESP = 41,
    CTRL_SCHEDS_REQ = 42,
    CTRL_SCHEDS_RESP = 43,
    CTRL_CHEM_REQ = 44
};

    // size lookup table for message types
    // MUST MATCH enum network_msg_typ_t
static constexpr size_t network_msg_typ_sizes[] = {
    sizeof(network_msg_ctrl_set_ack_t),                                  // 0: CTRL_SET_ACK
    sizeof(network_msg_ctrl_circuit_set_t),                              // 1: CTRL_CIRCUIT_SET
    0 /* sizeof(network_msg_ctrl_sched_req_t) returns 1, not 0 */,       // 2: CTRL_SCHED_REQ
    sizeof(network_msg_ctrl_sched_resp_t),                               // 3: CTRL_SCHED_RESP
    sizeof(network_msg_ctrl_state_bcast_t),                              // 4: CTRL_STATE_BCAST
    0 /* sizeof(network_msg_ctrl_time_req_t) returns 1, not 0 */,        // 5: CTRL_TIME_REQ
    sizeof(network_msg_ctrl_time_resp_t),                                // 6: CTRL_TIME_RESP
    sizeof(network_msg_ctrl_time_set_t),                                 // 7: CTRL_TIME_SET
    0 /* sizeof(network_msg_ctrl_heat_req_t) returns 1, not 0 */,        // 8: CTRL_HEAT_REQ
    sizeof(network_msg_ctrl_heat_resp_t),                                // 9: CTRL_HEAT_RESP
    sizeof(network_msg_ctrl_heat_set_t),                                 // 10: CTRL_HEAT_SET
    0 /* sizeof(network_msg_ctrl_layout_req_t) returns 1, not 0 */,      // 11: CTRL_LAYOUT_REQ
    sizeof(network_msg_ctrl_layout_resp_t),                              // 12: CTRL_LAYOUT_RESP
    sizeof(network_msg_ctrl_layout_set_t),                               // 13: CTRL_LAYOUT_SET
    sizeof(network_msg_pump_reg_set_t),                                  // 14: PUMP_REG_SET
    sizeof(network_msg_pump_reg_resp_t),                                 // 15: PUMP_REG_RESP
    sizeof(network_msg_pump_ctrl_t),                                     // 16: PUMP_CTRL_SET
    sizeof(network_msg_pump_ctrl_t),                                     // 17: PUMP_CTRL_RESP
    sizeof(network_msg_pump_mode_t),                                     // 18: PUMP_MODE_SET
    sizeof(network_msg_pump_mode_t),                                     // 19: PUMP_MODE_RESP
    sizeof(network_msg_pump_run_t),                                      // 20: PUMP_RUN_SET
    sizeof(network_msg_pump_run_t),                                      // 21: PUMP_RUN_RESP
    0 /* sizeof(network_msg_pump_status_req_t) returns 1, not 0 */,      // 22: PUMP_STATUS_REQ
    sizeof(network_msg_pump_status_resp_t),                              // 23: PUMP_STATUS_RESP
    sizeof(network_msg_chlor_ping_req_t),                                // 24: CHLOR_PING_REQ
    sizeof(network_msg_chlor_ping_resp_t),                               // 25: CHLOR_PING_RESP
    sizeof(network_msg_chlor_name_resp_t),                               // 26: CHLOR_NAME_RESP
    sizeof(network_msg_chlor_level_set_t),                               // 27: CHLOR_LEVEL_SET
    sizeof(network_msg_chlor_level_resp_t),                              // 28: CHLOR_LEVEL_RESP
    sizeof(network_msg_chlor_name_req_t),                                // 29: CHLOR_NAME_REQ
    0 /* sizeof(network_msg_ctrl_valve_req_t) returns 1, not 0 */,       // 30: CTRL_VALVE_REQ
    sizeof(network_msg_ctrl_valve_resp_t),                               // 31: CTRL_VALVE_RESP
    0 /* sizeof(network_msg_ctrl_version_req_t) returns 1, not 0 */,     // 32: CTRL_VERSION_REQ
    sizeof(network_msg_ctrl_version_resp_t),                             // 33: CTRL_VERSION_RESP
    0 /* sizeof(network_msg_ctrl_solarpump_req_t) returns 1, not 0 */,   // 34: CTRL_SOLARPUMP_REQ
    sizeof(network_msg_ctrl_solarpump_resp_t),                           // 35: CTRL_SOLARPUMP_RESP
    0 /* sizeof(network_msg_ctrl_delay_req_t) returns 1, not 0 */,       // 36: CTRL_DELAY_REQ
    sizeof(network_msg_ctrl_delay_resp_t),                               // 37: CTRL_DELAY_RESP
    0 /* sizeof(network_msg_ctrl_heat_setpt_req_t) returns 1, not 0 */,  // 38: CTRL_HEAT_SETPT_REQ
    sizeof(network_msg_ctrl_heat_setpt_resp_t),                          // 39: CTRL_HEAT_SETPT_RESP
    sizeof(network_msg_ctrl_circ_names_req_t),                           // 40: CTRL_CIRC_NAMES_REQ
    sizeof(network_msg_ctrl_circ_names_resp_t),                          // 41: CTRL_CIRC_NAMES_RESP
    sizeof(network_msg_ctrl_scheds_req_t),                               // 42: CTRL_SCHEDS_REQ
    sizeof(network_msg_ctrl_scheds_resp_t),                              // 43: CTRL_SCHEDS_RESP
    sizeof(network_msg_ctrl_chem_req_t)                                  // 44: CTRL_CHEM_REQ
};

inline esp_err_t
network_msg_typ_get_size(network_msg_typ_t typ, size_t * size)
{
    uint8_t idx = static_cast<uint8_t>(typ);
    if (idx < ARRAY_SIZE(network_msg_typ_sizes)) {
        *size = network_msg_typ_sizes[idx];
        return ESP_OK;
    }
    return ESP_FAIL;
}

    // structure to hold message type metadata
struct network_msg_typ_info_t {
    datalink_prot_t  proto;
    datalink_typ_t   typ;
    
    constexpr network_msg_typ_info_t(datalink_prot_t p, uint8_t pt) 
        : proto(p), typ{.raw = pt} {}
    
    constexpr network_msg_typ_info_t(datalink_prot_t p, datalink_typ_ctrl_t ct)
        : proto(p), typ{.ctrl = ct} {}
    
    constexpr network_msg_typ_info_t(datalink_prot_t p, datalink_typ_pump_t pt)
        : proto(p), typ{.pump = pt} {}
    
    constexpr network_msg_typ_info_t(datalink_prot_t p, datalink_typ_chlor_t ct)
        : proto(p), typ{.chlor = ct} {}
};

    // maps {datalink_prot and datalink_typ_t} to network_msg_typ_t
    // MUST MATCH network_msg_typ_t
constexpr network_msg_typ_info_t network_msg_typ_info[] = {
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SET_ACK},          // 0: CTRL_SET_ACK
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::CIRCUIT_SET},      // 1: CTRL_CIRCUIT_SET
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SCHED_REQ},        // 2: CTRL_SCHED_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SCHED_RESP},       // 3: CTRL_SCHED_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::STATE_BCAST},      // 4: CTRL_STATE_BCAST
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::TIME_REQ},         // 5: CTRL_TIME_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::TIME_RESP},        // 6: CTRL_TIME_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::TIME_SET},         // 7: CTRL_TIME_SET
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::HEAT_REQ},         // 8: CTRL_HEAT_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::HEAT_RESP},        // 9: CTRL_HEAT_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::HEAT_SET},         // 10: CTRL_HEAT_SET
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::LAYOUT_REQ},       // 11: CTRL_LAYOUT_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::LAYOUT_RESP},      // 12: CTRL_LAYOUT_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::LAYOUT_SET},       // 13: CTRL_LAYOUT_SET
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::REG},              // 14: PUMP_REG_SET
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::REG},              // 15: PUMP_REG_RESP
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::CTRL},             // 16: PUMP_CTRL_SET
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::CTRL},             // 17: PUMP_CTRL_RESP
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::MODE},             // 18: PUMP_MODE_SET
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::MODE},             // 19: PUMP_MODE_RESP
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::RUN},              // 20: PUMP_RUN_SET
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::RUN},              // 21: PUMP_RUN_RESP
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::STATUS},           // 22: PUMP_STATUS_REQ
    {datalink_prot_t::A5_PUMP, datalink_typ_pump_t::STATUS},           // 23: PUMP_STATUS_RESP
    {datalink_prot_t::IC,      datalink_typ_chlor_t::PING_REQ},        // 24: CHLOR_PING_REQ
    {datalink_prot_t::IC,      datalink_typ_chlor_t::PING_RESP},       // 25: CHLOR_PING_RESP
    {datalink_prot_t::IC,      datalink_typ_chlor_t::NAME_RESP},       // 26: CHLOR_NAME_RESP
    {datalink_prot_t::IC,      datalink_typ_chlor_t::LEVEL_SET},       // 27: CHLOR_LEVEL_SET
    {datalink_prot_t::IC,      datalink_typ_chlor_t::LEVEL_RESP},      // 28: CHLOR_LEVEL_RESP
    {datalink_prot_t::IC,      datalink_typ_chlor_t::NAME_REQ},        // 29: CHLOR_NAME_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::VALVE_REQ},        // 30: CTRL_VALVE_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::VALVE_RESP},       // 31: CTRL_VALVE_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::VERSION_REQ},      // 32: CTRL_VERSION_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::VERSION_RESP},     // 33: CTRL_VERSION_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SOLARPUMP_REQ},    // 34: CTRL_SOLARPUMP_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SOLARPUMP_RESP},   // 35: CTRL_SOLARPUMP_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::DELAY_REQ},        // 36: CTRL_DELAY_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::DELAY_RESP},       // 37: CTRL_DELAY_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::HEAT_SETPT_REQ},   // 38: CTRL_HEAT_SETPT_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::HEAT_SETPT_RESP},  // 39: CTRL_HEAT_SETPT_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::CIRC_NAMES_REQ},   // 40: CTRL_CIRC_NAMES_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::CIRC_NAMES_RESP},  // 41: CTRL_CIRC_NAMES_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SCHEDS_REQ},       // 42: CTRL_SCHEDS_REQ
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::SCHEDS_RESP},      // 43: CTRL_SCHEDS_RESP
    {datalink_prot_t::A5_CTRL, datalink_typ_ctrl_t::CHEM_REQ}          // 44: CTRL_CHEM_REQ
};

inline const network_msg_typ_info_t *
network_msg_typ_get_info(network_msg_typ_t typ)
{
    uint8_t idx = static_cast<uint8_t>(typ);
    if (idx < ARRAY_SIZE(network_msg_typ_info)) {
        return &network_msg_typ_info[idx];
    }
    return nullptr;
}


/**
 * @brief    Represents a generic network message for the Pentair protocol.
 *
 * @details
 * Contains the message type and a union of all possible protocol-specific message data 
 * structures, allowing flexible handling of controller, pump, and chlorinator messages.
 *
 * @var typ  The network message type identifier.
 * @var u    Union containing all supported message data structures for A5/controller, A5/pump, and IC messages.
 */

struct network_msg_t {
    network_msg_typ_t typ;
    union network_msg_union_t {
        network_msg_pump_reg_set_t  pump_reg_set;
        network_msg_pump_reg_resp_t  pump_reg_set_resp;
        network_msg_pump_ctrl_t  pump_ctrl;
        network_msg_pump_mode_t  pump_mode;
        network_msg_pump_run_t  pump_run;
        network_msg_pump_status_resp_t  pump_status_resp;
        network_msg_ctrl_set_ack_t ctrl_set_ack;
        network_msg_ctrl_circuit_set_t  ctrl_circuit_set;
        network_msg_ctrl_sched_resp_t  ctrl_sched_resp;
        network_msg_ctrl_state_bcast_t  ctrl_state;
        network_msg_ctrl_time_set_t  ctrl_time_set;
        network_msg_ctrl_time_resp_t  ctrl_time_resp;
        network_msg_ctrl_heat_resp_t  ctrl_heat_resp;
        network_msg_ctrl_heat_set_t  ctrl_heat_set;
        network_msg_ctrl_layout_resp_t  ctrl_layout_resp;
        network_msg_ctrl_layout_set_t  ctrl_layout_set;
        network_msg_ctrl_version_req_t  ctrl_version_req;
        network_msg_ctrl_version_resp_t  ctrl_version_resp;
        network_msg_ctrl_valve_req_t  ctrl_valve_req;
        network_msg_ctrl_valve_resp_t  ctrl_valve_resp;
        network_msg_ctrl_solarpump_req_t  ctrl_solarpump_req;
        network_msg_ctrl_solarpump_resp_t  ctrl_solarpump_resp;
        network_msg_ctrl_delay_req_t  ctrl_delay_req;
        network_msg_ctrl_delay_resp_t  ctrl_delay_resp;
        network_msg_ctrl_heat_setpt_req_t  ctrl_heat_set_req;
        network_msg_ctrl_heat_setpt_resp_t  ctrl_heat_set_resp;
        network_msg_ctrl_circ_names_req_t  ctrl_circ_names_req;
        network_msg_ctrl_circ_names_resp_t  ctrl_circ_names_resp;
        network_msg_ctrl_chem_req_t  ctrl_chem_req;
        network_msg_ctrl_scheds_req_t  ctrl_scheds_req;
        network_msg_ctrl_scheds_resp_t  ctrl_scheds_resp;
        network_msg_chlor_ping_req_t  chlor_ping_req;
        network_msg_chlor_ping_resp_t  chlor_ping_resp;
        network_msg_chlor_name_resp_t  chlor_name_resp;
        network_msg_chlor_level_set_t  chlor_level_set;
        network_msg_chlor_level_resp_t  chlor_level_resp;
        network_msg_chlor_name_req_t  chlor_name_req;
        uint8_t bytes[0];  // access union as bytes
    } u;
};

constexpr size_t NETWORK_MSG_TYP_SIZES_COUNT = std::size(network_msg_typ_sizes);
constexpr size_t NETWORK_MSG_TYP_INFO_COUNT = std::size(network_msg_typ_info);

#ifndef __INTELLISENSE__
static_assert(NETWORK_MSG_TYP_SIZES_COUNT == enum_count<network_msg_typ_t>());  // IntelliSense doesn't evaluate constexpr functions, use temporary constant
static_assert(NETWORK_MSG_TYP_INFO_COUNT == enum_count<network_msg_typ_t>());   // IntelliSense doesn't evaluate constexpr functions, use temporary constant
#endif

}  // namespace opnpool
}  // namespace esphome