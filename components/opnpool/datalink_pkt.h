#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"
#include "skb.h"

namespace esphome {
namespace opnpool {

enum class datalink_prot_t : uint8_t {
  IC = 0x00,
  A5_CTRL = 0x01,
  A5_PUMP = 0x02,
  NONE = 0xFF,
};

typedef uint8_t datalink_data_t;

/**
 * State info retained between successive datalink_rx() calls
 **/

typedef struct datalink_pkt_t {
    datalink_prot_t    prot;      // datalink_prot as detected by `_read_head`
    uint8_t            prot_typ;  // from datalink_hdr_a5->typ
    uint8_t            src;       // from datalink_hdr_a5->src
    uint8_t            dst;       // from datalink_hdr_a5->dst
    datalink_data_t *  data;
    size_t             data_len;
    skb_handle_t       skb;
} datalink_pkt_t;

/* Helper functions for datalink_prot_t */
inline const char *
datalink_prot_str(datalink_prot_t const prot)
{
    auto name = magic_enum::enum_name(prot);
    if (!name.empty()) {
        return name.data();
    }
    static char buf[8];
    snprintf(buf, sizeof(buf), "0x%02X", static_cast<uint8_t>(prot));
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