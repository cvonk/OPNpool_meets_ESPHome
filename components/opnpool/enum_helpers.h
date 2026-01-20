#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/log.h>

#include "magic_enum.h"
#include "to_str.h"

namespace esphome {
namespace opnpool {

static char const * const ENUM_HELPER_TAG = "enum_helpers";

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
        // try magic_enum first for efficient lookup
    auto value = magic_enum::enum_cast<EnumT>(std::string_view(enum_str), magic_enum::case_insensitive);
    if (value.has_value()) {
        return static_cast<int>(value.value());
    }
        // fallback: search through entire uint8_t range if not found in enum
    for (uint16_t ii = 0; ii <= 0xFF; ii++) {
        auto candidate = static_cast<EnumT>(ii);
        auto name = magic_enum::enum_name(candidate);
        if (!name.empty() && strcasecmp(enum_str, name.data()) == 0) {
            return ii;
        }
    }
    ESP_LOGE(ENUM_HELPER_TAG, "enum_str '%s' not found", enum_str);
    return -1;
}

    // helper to return the total number of enum values
template<typename EnumT>
constexpr size_t enum_count() {
    return magic_enum::enum_count<EnumT>();
}

    // helper to to return the index (underlying value) of an enum
template <typename E>
constexpr auto
enum_index(E e) noexcept -> std::underlying_type_t<E>
{
    static_assert(std::is_enum<E>::value, "E must be an enum type");
    return static_cast<std::underlying_type_t<E>>(e);
}

} // namespace opnpool
} // namespace esphome