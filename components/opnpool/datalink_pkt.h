/**
 * @file datalink_pkt.h
 * @author Coert Vonk (@cvonk on GitHub)
 * @brief OPNpool - Defines the core data structure for the OPNpool data link layer.
 * 
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * 
 * @details
 * This header defines the core data structures, enums, and utility functions for the OPNpool data link layer.
 * The file defines the main datalink packet structure used for encapsulating protocol messages, along with macros
 * for alignment and packing to ensure compatibility with embedded hardware.
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
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <strings.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"
#include "skb.h"

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif
#ifndef ALIGN
# define ALIGN(type) __attribute__((aligned( __alignof__(type)) ))
#endif
#ifndef PACK
# define PACK(type)  __attribute__((aligned( __alignof__(type)), packed))
# define PACK8       __attribute__((aligned( __alignof__(uint8_t)), packed))
#endif

namespace esphome {
namespace opnpool {

enum class datalink_prot_t : uint8_t {
    IC      = 0x00,
    A5_CTRL = 0x01,
    A5_PUMP = 0x02,
    NONE    = 0xFF,
};

/**
 * @brief Controller messages types
 */

enum class datalink_typ_ctrl_t : uint8_t {
    SET_ACK         = 0x01,
    STATE_BCAST     = 0x02,
    CANCEL_DELAY    = 0x03,
    TIME_RESP       = 0x05,
    TIME_SET        = 0x85,
    TIME_REQ        = 0xC5,
    CIRCUIT_RESP    = 0x06,
    CIRCUIT_SET     = 0x86,
    CIRCUIT_REQ     = 0xC6,
    HEAT_RESP       = 0x08,
    HEAT_SET        = 0x88,
    HEAT_REQ        = 0xC8,
    HEAT_PUMP_RESP  = 0x10,
    HEAT_PUMP_SET   = 0x90,
    HEAT_PUMP_REQ   = 0xD0,
    SCHED_RESP      = 0x1E,
    SCHED_SET       = 0x9E,
    SCHED_REQ       = 0xDE,
    LAYOUT_RESP     = 0x21,
    LAYOUT_SET      = 0xA1,
    LAYOUT_REQ      = 0xE1,
    CIRC_NAMES_RESP = 0x0B,
    CIRC_NAMES_REQ  = 0xCB,
    SCHEDS_RESP     = 0x11,
    SCHEDS_REQ      = 0xD1,
    CHEM_RESP       = 0x12,
    CHEM_REQ        = 0xD2,
    VALVE_RESP      = 0x1D,
    VALVE_REQ       = 0xDD,
    SOLARPUMP_RESP  = 0x22,
    SOLARPUMP_REQ   = 0xE2,
    DELAY_RESP      = 0x23,
    DELAY_REQ       = 0xE3,
    HEAT_SETPT_RESP = 0x28,
    HEAT_SETPT_REQ  = 0xE8,
    VERSION_RESP    = 0xFC,
    VERSION_REQ     = 0xFD
};

inline const char *
datalink_typ_ctrl_str(datalink_typ_ctrl_t const typ_ctrl)
{
    auto name = magic_enum::enum_name(typ_ctrl);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[3];
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(typ_ctrl));
    return buf;
}


/**
 * @brief Pump message types
 */

enum class datalink_typ_pump_t : uint8_t {
    REG        = 0x01,
    CTRL       = 0x04,
    MODE       = 0x05,
    RUN        = 0x06,
    STATUS     = 0x07,
    UNKNOWN_FF = 0xFF
};

inline const char *
datalink_typ_pump_str(datalink_typ_pump_t const pump)
{
    auto name = magic_enum::enum_name(pump);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[3];
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(pump));
    return buf;
}

/**
 * @brief Chlorinator message types
 */

enum class datalink_typ_chlor_t : uint8_t {
    PING_REQ   = 0x00,
    PING_RESP  = 0x01,
    NAME_RESP  = 0x03,
    LEVEL_SET  = 0x11,
    LEVEL_RESP = 0x12,
    NAME_REQ   = 0x14
};

inline const char *
datalink_typ_chlor_str(datalink_typ_chlor_t const chlor)
{
    auto name = magic_enum::enum_name(chlor);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[3];
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(chlor));
    return buf;
}

typedef union datalink_typ_t {
    datalink_typ_ctrl_t  ctrl;
    datalink_typ_pump_t  pump;
    datalink_typ_chlor_t chlor;
    uint8_t              raw;
} PACK8 datalink_typ_t;


typedef uint8_t datalink_address;
typedef uint8_t datalink_data_t;

/**
 * @brief Data link packet structure
 */

typedef struct datalink_pkt_t {
    datalink_prot_t    prot;      // datalink_prot as detected by `_read_head`
    datalink_typ_t     typ;       // from datalink_hdr_a5->typ
    datalink_address   src;       // from datalink_hdr_a5->src
    datalink_address   dst;       // from datalink_hdr_a5->dst
    datalink_data_t *  data;
    size_t             data_len;
    skb_handle_t       skb;
} datalink_pkt_t;

inline const char *
datalink_prot_str(datalink_prot_t const prot)
{
    auto name = magic_enum::enum_name(prot);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[3];
    snprintf(buf, sizeof(buf), "%02X", static_cast<uint8_t>(prot));
    return buf;
}

inline int
datalink_prot_nr(char const * const prot_str)
{
    if (!prot_str) {
        return -1;
    }
    
        // try magic_enum first for efficient lookup
    auto value = magic_enum::enum_cast<datalink_prot_t>(std::string_view(prot_str), magic_enum::case_insensitive);
    if (value.has_value()) {
        return static_cast<int>(value.value());
    }
    
        // search through entire uint8_t range (0-255) if not found in enum
    for (uint16_t ii = 0; ii <= 0xFF; ii++) {
        auto candidate = static_cast<datalink_prot_t>(ii);
        auto name = magic_enum::enum_name(candidate);
        if (!name.empty() && strcasecmp(prot_str, name.data()) == 0) {
            return ii;
        }
    }    
    return -1;
}

} // namespace opnpool
} // namespace esphome