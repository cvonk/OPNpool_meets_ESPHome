/**
 * @file rs485.cpp
 * @brief RS485 driver: receive/sent bytes to/from the RS485 transceiver
 *
 * @details
 * This file implements the RS485 hardware driver for the OPNpool component, providing
 * low-level functions to initialize, configure, and operate the RS485 transceiver on
 * ESPHome-supported hardware. It handles UART setup for half-duplex communication, GPIO
 * configuration for transmit/receive direction, and manages a transmit queue for outgoing
 * packets. The driver exposes a handle with function pointers for higher-level protocol
 * layers to interact with the RS485 interface, ensuring reliable and efficient
 * communication with pool equipment over the RS485 bus.
 *
 * The design assumes a single-threaded environment (as provided by ESPHome), so no
 * explicit thread safety is implemented. 
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_types.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esphome/core/log.h>
#include <esp_rom_sys.h>
#include <string.h>

#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "rs485";

static size_t     _rxBufSize = 127;
static TickType_t _rxTimeout = (100 / portTICK_PERIOD_MS);
static TickType_t _txTimeout = (100 / portTICK_PERIOD_MS);

static uart_port_t _uart_port;
static gpio_num_t _flow_control_pin;

/**
 * @brief Returns the number of bytes available in the UART RX buffer.
 */
static int
_available()
{
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(_uart_port, (size_t *) &length));
    return length;
}

/**
 * @brief     Reads bytes from the UART RX buffer with a timeout.
 *
 * @param dst Destination buffer.
 * @param len Number of bytes to read.
 * @return    Number of bytes read.
 */
static int
_read_bytes(uint8_t * dst, uint32_t len)
{
    return uart_read_bytes(_uart_port, dst, len, _rxTimeout);
}

/**
 * @brief     Writes bytes to the UART TX buffer.
 *
 * @param src Source buffer.
 * @param len Number of bytes to write.
 * @return    Number of bytes written.
 */
static int
_write_bytes(uint8_t * src, size_t len)
{
    return uart_write_bytes(_uart_port, (char *) src, len);
}

/**
 * @brief Flushes the UART TX and RX buffers, waiting for TX to complete.
 */
static void
_flush(void)
{
    ESP_ERROR_CHECK(uart_wait_tx_done(_uart_port, _txTimeout));
    ESP_ERROR_CHECK(uart_flush_input(_uart_port));
}

/**
 * @brief        Queues a packet for transmission on the RS-485 bus.
 *
 * @param handle RS-485 handle.
 * @param pkt    Packet to queue.
 */
static void
_queue(rs485_handle_t const handle, datalink_pkt_t const * const pkt)
{
    if (pkt != nullptr) {
        rs485_q_msg_t msg = {
            .pkt = pkt,
        };
        if (xQueueSendToBack(handle->tx_q, &msg, 0) != pdPASS) {
            ESP_LOGE(TAG, "tx_q full");
            free(pkt->skb);
            free((void *) pkt);
        }
    }
}

/**
 * @brief        Dequeues a packet from the RS-485 transmit queue.
 *
 * @param handle RS-485 handle.
 * @return       Pointer to the dequeued packet, or NULL if none available.
 */
static datalink_pkt_t const *
_dequeue(rs485_handle_t const handle)
{
    rs485_q_msg_t msg{};
    if (xQueueReceive(handle->tx_q, &msg, (TickType_t)0) == pdPASS) {
        if (msg.pkt == nullptr) {
            ESP_LOGE(TAG, "Dequeued packet is null");
        }
        return msg.pkt;
    }
    return NULL;
}

/**
 * @brief           Sets the RS-485 transceiver to transmit or receive mode.
 *
 * @param tx_enable True to enable transmit mode, false for receive mode.
 */
static void
_tx_mode(bool const tx_enable)
{
	// messages should be sent directly after an A5 packets (and before any IC packets)
	// 2BD: there might be a mandatory wait after enabling this pin !!!!!!! A few words on
	// the DE signal:
    //  - choose a GPIO that doesn't mind being pulled down during reset

    if (tx_enable) {
        gpio_set_level(_flow_control_pin, 1);  // enable RS485 transmit DE=1 and RE*=1 (DE=driver enable, RE*=inverted receive enable)
    } else {
        _flush();  // wait until last byte starts transmitting
        esp_rom_delay_us(1500);  // wait until last byte is transmitted (10 bits / 9600 baud =~ 1042 ms)
        gpio_set_level(_flow_control_pin, 0);  // enable RS485 receive
     }
}

/**
 * @brief Initializes the RS485 hardware interface and driver.
 *
 * @details
 * Configures the specified UART port and GPIO pins for RS485 half-duplex communication,
 * sets up the UART parameters (baud rate, data bits, stop bits, etc.), and initializes
 * the flow control pin for transmit/receive direction control. Allocates and initializes
 * the RS485 handle structure, sets up the transmit queue, and assigns function pointers
 * for RS485 operations. Returns a handle to the initialized RS485 interface for use by
 * higher-level protocol layers.
 *
 * @param rs485_pins Pointer to the structure containing RX, TX, and flow control pin
 * numbers.
 * @return rs485_handle_t Handle to the initialized RS485 interface.
 */
rs485_handle_t
rs485_init(rs485_pins_t const * const rs485_pins)
{
    gpio_num_t const rx_pin = static_cast<gpio_num_t>(rs485_pins->rx_pin);
    gpio_num_t const tx_pin = static_cast<gpio_num_t>(rs485_pins->tx_pin);
    _flow_control_pin = static_cast<gpio_num_t>(rs485_pins->flow_control_pin);

    _uart_port = static_cast<uart_port_t>(2);  // UART2

    uart_config_t const uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << static_cast<uint8_t>(_flow_control_pin)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK( gpio_config(&io_conf) );
    gpio_set_level(_flow_control_pin, 0);

    ESP_LOGI(TAG, "Initializing RS485 on UART%u (RX pin %u, TX pin %u, RE/DE pin %u) ..",
             _uart_port, rx_pin, tx_pin, _flow_control_pin);

    uart_param_config(_uart_port, &uart_config);
    uart_set_pin(_uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(_uart_port, _rxBufSize * 2, 0, 0, NULL, 0);  // no tx buffer
    uart_set_mode(_uart_port, UART_MODE_RS485_HALF_DUPLEX);

    QueueHandle_t const tx_q = xQueueCreate(5, sizeof(rs485_q_msg_t));
    if (tx_q == nullptr) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return nullptr;
    }

    rs485_handle_t handle = static_cast<rs485_handle_t>(calloc(1, sizeof(rs485_instance_t)));
    if (handle == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate RS485 handle");
        return nullptr;
    }

    handle->available = _available;
    handle->read_bytes = _read_bytes;
    handle->write_bytes = _write_bytes;
    handle->flush = _flush;
    handle->tx_mode = _tx_mode;
    handle->queue = _queue;
    handle->dequeue = _dequeue;
    handle->tx_q = tx_q;
    
    _tx_mode(false);

    return handle;
}

}  // namespace opnpool
}  // namespace esphome