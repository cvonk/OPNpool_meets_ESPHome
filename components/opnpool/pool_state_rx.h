#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include "network_msg.h"
#include "pool_state.h"

namespace esphome {
namespace opnpool {
namespace pool_state_rx {

esp_err_t update_state(network_msg_t const * const msg, poolstate_t * const state);

}  // namespace pool_state_rx
}  // namespace opnpool
}  // namespace esphome