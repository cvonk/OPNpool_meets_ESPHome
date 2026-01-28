#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <driver/uart.h>
#include <cstddef>

#include "ipc.h"

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct datalink_pkt_t;
using rs485_handle_t = struct rs485_instance_t *;

using rs485_available_fnc_t   = int (*)(void);
using rs485_read_bytes_fnc_t  = int (*)(uint8_t * dst, uint32_t len);
using rs485_write_bytes_fnc_t = int (*)(uint8_t * src, size_t len);
using rs485_write_fnc_t       = int (*)(uint8_t src);
using rs485_flush_fnc_t       = void (*)(void);
using rx485_tx_mode_fnc_t     = void (*)(bool const tx_enable);
using rs485_queue_fnc_t       = void (*)(rs485_handle_t const handle, datalink_pkt_t const * const pkt);
using rs485_dequeue_fnc_t     = datalink_pkt_t const * (*)(rs485_handle_t const handle);

struct rs485_instance_t {
    rs485_available_fnc_t available;      // bytes available in rx buffer
    rs485_read_bytes_fnc_t read_bytes;    // read bytes from rx buffer
    rs485_write_bytes_fnc_t write_bytes;  // write bytes to tx buffer 
    rs485_flush_fnc_t flush;              // wait until all bytes are transmitted
    rx485_tx_mode_fnc_t tx_mode;          // controls RTS pin (for half-duplex)
    rs485_queue_fnc_t queue;              // queue to handle->tx_q
    rs485_dequeue_fnc_t dequeue;          // dequeue from handle->tx_q
    QueueHandle_t tx_q;                   // transmit queue
};

struct  rs485_q_msg_t {
    datalink_pkt_t const * const pkt;
};


    // rs485.cpp
[[nodiscard]] rs485_handle_t rs485_init(rs485_pins_t const * const rs485_pins);

} // namespace opnpool
} // namespace esphome