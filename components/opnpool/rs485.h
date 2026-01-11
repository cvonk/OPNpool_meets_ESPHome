#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <cstddef>
#include <esp_system.h>
#include <esp_types.h>
#include <driver/uart.h>

#include "ipc.h"

namespace esphome {
namespace opnpool {

    // forward declarations
struct datalink_pkt_t;
typedef struct rs485_instance_t * rs485_handle_t;

typedef int (* rs485_available_fnc_t)(void);
typedef int (* rs485_read_bytes_fnc_t)(uint8_t * dst, uint32_t len);
typedef int (* rs485_write_bytes_fnc_t)(uint8_t * src, size_t len);
typedef int (* rs485_write_fnc_t)(uint8_t src);
typedef void (* rs485_flush_fnc_t)(void);
typedef void (* rx485_tx_mode_fnc_t)(bool const tx_enable);
typedef void (* rs485_queue_fnc_t)(rs485_handle_t const handle, datalink_pkt_t const * const pkt);
typedef datalink_pkt_t const * (* rs485_dequeue_fnc_t)(rs485_handle_t const handle);

typedef struct rs485_instance_t {
    rs485_available_fnc_t available;      // bytes available in rx buffer
    rs485_read_bytes_fnc_t read_bytes;    // read bytes from rx buffer
    rs485_write_bytes_fnc_t write_bytes;  // write bytes to tx buffer 
    rs485_flush_fnc_t flush;              // wait until all bytes are transmitted
    rx485_tx_mode_fnc_t tx_mode;          // controls RTS pin (for half-duplex)
    rs485_queue_fnc_t queue;              // queue to handle->tx_q
    rs485_dequeue_fnc_t dequeue;          // dequeue from handle->tx_q
    QueueHandle_t tx_q;                   // transmit queue
} rs485_instance_t;

typedef struct  rs485_q_msg_t {
    datalink_pkt_t const * const pkt;
} rs485_q_msg_t;


    // rs485.cpp
rs485_handle_t rs485_init(rs485_pins_t const * const rs485_pins);

} // namespace opnpool
} // namespace esphome