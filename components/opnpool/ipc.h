#ifndef __cplusplus
# error "This header requires C++ compilation"
#endif

#pragma once

#include <esp_types.h>
#include <freertos/queue.h>

#include "skb.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))
#ifndef MIN
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#endif
#ifndef MAX
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#endif

    // ipc_t passed to tasks

typedef struct rs485_pins_t {
    uint8_t rx_pin{25};
    uint8_t tx_pin{26};
    uint8_t flow_control_pin{27};
} rs485_pins_t;

typedef struct config_t {
    rs485_pins_t rs485_pins;
} config_t;

typedef struct ipc_t {
    QueueHandle_t to_main_q;
    QueueHandle_t to_pool_q;
    config_t      config;
} ipc_t;

void ipc_send_network_msg_to_main_task(network_msg_t const * const network_msg, ipc_t const * const ipc);
void ipc_send_network_msg_to_pool_task(network_msg_t const * const network_msg, ipc_t const * const ipc);

} // namespace opnpool
} // namespace esphome