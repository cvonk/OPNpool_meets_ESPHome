#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace esphome {
namespace opnpool {

struct rs485_pins_t {
    uint8_t rx_pin{25};
    uint8_t tx_pin{26};
    uint8_t flow_control_pin{27};
};

struct config_t {
    rs485_pins_t rs485_pins;
};

struct ipc_t {
    QueueHandle_t to_main_q;
    QueueHandle_t to_pool_q;
    config_t      config;
};

    // forward declarations
struct network_msg_t;

void ipc_send_network_msg_to_main_task(network_msg_t const * const network_msg, ipc_t const * const ipc);
void ipc_send_network_msg_to_pool_task(network_msg_t const * const network_msg, ipc_t const * const ipc);

} // namespace opnpool
} // namespace esphome