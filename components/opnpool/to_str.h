#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include "magic_enum.h"

namespace esphome {
namespace opnpool {

    // Should be at least ((sizeof(datalink_hdr_t) + sizeof(network_msg_ctrl_state_bcast_t) + 1) * 3 + 50).
    // That 3 bytes for each hex value when displaying raw, and another 50 for displying date/time.
#define TO_STR_BUF_SIZE (200)

    // reusable global string
typedef struct name_str_t {
	char str[TO_STR_BUF_SIZE];  //  this is str.str[]
	uint_least8_t idx;
	char const * const noMem;
	char const * const digits;
} name_str_t;

    // helper to convert an boolean value to its string representation
char const * bool_str(bool const value);

    // helper to convert an uint value to its string representation
char const * uint_str(uint16_t const value);

    // helper to convert an uint8_t value to its string representation
char const * uint8_str(uint8_t const value);

    // helper to convert an uint16_t value to its string representation
char const * uint16_str(uint16_t const value);

    // helper to convert an enum value to its string representation
template<typename EnumT>
inline const char * enum_str(EnumT value)
{
    auto name = magic_enum::enum_name(value);
    if (!name.empty()) {
        return name.data();
    }
    return uint8_str(static_cast<uint8_t>(value));  // fallback
}

    // helper to convert a string to its enum value representation
template<typename EnumT>
inline int enum_nr(char const * const enum_str)
{
    if (!enum_str) {
        return -1;
    }
    // Try magic_enum first for efficient lookup
    auto value = magic_enum::enum_cast<EnumT>(std::string_view(enum_str), magic_enum::case_insensitive);
    if (value.has_value()) {
        return static_cast<int>(value.value());
    }
    // Fallback: search through entire uint8_t range if not found in enum
    for (uint16_t ii = 0; ii <= 0xFF; ii++) {
        auto candidate = static_cast<EnumT>(ii);
        auto name = magic_enum::enum_name(candidate);
        if (!name.empty() && strcasecmp(enum_str, name.data()) == 0) {
            return ii;
        }
    }
    return -1;
}

    // helper to return the number of enum values
template<typename EnumT>
constexpr size_t enum_count() {
    return magic_enum::enum_count<EnumT>();
}


    // management
void name_reset_idx(void);  // should be called periodically. e.g. before work on a packet starts.
extern name_str_t name_str;

} // namespace opnpool
} // namespace esphome