#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include "network_msg.h"
#include "opnpool_state.h"

namespace esphome {
namespace opnpool {
namespace opnpool_state_rx {

esp_err_t update(network_msg_t const * const msg, poolstate_t * const state);

}  // namespace opnpool_state_rx
}  // namespace opnpool
}  // namespace esphome