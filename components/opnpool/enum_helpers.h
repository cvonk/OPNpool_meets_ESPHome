#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include "magic_enum.h"

namespace esphome {
namespace opnpool {

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

} // namespace opnpool
} // namespace esphome