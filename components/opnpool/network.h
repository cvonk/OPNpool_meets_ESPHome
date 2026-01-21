/**
 * @file network.h
 * @brief Network Layer Definitions for OPNpool component
 *
 * @details
 * This header defines the interface for the network layer of the OPNpool component. The
 * network layer abstracts protocol translation and message construction, enabling
 * reliable communication between the ESP32 and pool controller over RS-485.
 *
 * The network layer provides two main functions:
 * 1. `network_rx_msg()`: overlays a raw datalink packet with a network message structure.
 * 2. `network_create_pkt()`: Creates a datalink packet from a network message.
 *
 * The design supports multiple protocol variants and is intended for use in a
 * single-threaded ESPHome environment. Forward declarations are used to avoid
 * circular dependencies.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct datalink_pkt_t;
struct network_msg_t;

    // function prototypes for network_rx.cpp
esp_err_t network_rx_msg(datalink_pkt_t const * const pkt, network_msg_t * const msg, bool * const txOpportunity);

    // function prototypes for network_create.cpp
esp_err_t network_create_pkt(network_msg_t const * const msg, datalink_pkt_t * const pkt);

}  // namespace opnpool
}  // namespace esphome