#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct network_msg_t;
struct poolstate_t;

namespace poolstate_rx {

esp_err_t update_state(network_msg_t const * const msg, poolstate_t * const state);

}  // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome