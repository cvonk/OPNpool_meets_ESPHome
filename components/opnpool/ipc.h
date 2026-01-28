#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif


#include <esp_system.h>
#include <esp_types.h>
#include <freertos/queue.h>

#include "opnpool.h" // for rs485_pins_t

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct ipc_t;
struct network_msg_t;

struct config_t {
    rs485_pins_t rs485_pins;
};

struct ipc_t {
    QueueHandle_t to_main_q;
    QueueHandle_t to_pool_q;
    config_t      config;
};

    // function prototypes for ipc.cpp
void ipc_send_network_msg_to_main_task(network_msg_t const * const network_msg, ipc_t const * const ipc);
void ipc_send_network_msg_to_pool_task(network_msg_t const * const network_msg, ipc_t const * const ipc);

} // namespace opnpool
} // namespace esphome