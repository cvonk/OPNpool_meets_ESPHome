/**
 * @brief OPNpool - Inter Process Communication: mailbox messages exchanged between the tasks
 *
 * © Copyright 2014, 2019, 2022, Coert Vonk
 * 
 * This file is part of OPNpool.
 * OPNpool is free software: you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 * OPNpool is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with OPNpool. 
 * If not, see <https://www.gnu.org/licenses/>.
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: Copyright 2014,2019,2022 Coert Vonk
 */

#include <string.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esphome/core/log.h>

#include "utils.h"
#include "ipc.h"
#include "skb.h"

namespace esphome {
namespace opnpool {

static char const * const TAG = "ipc";

  /* X-Macro pattern keeps enums and strings synchronized */

static const char * const _ipc_to_home_typs[] = {
#define XX(num, name) #name,
  IPC_TO_HOME_TYP_MAP(XX)
#undef XX
};

const char *
ipc_to_home_typ_str(ipc_to_home_typ_t const typ)
{
    return ELEM_AT(_ipc_to_home_typs, typ, hex8_str(typ));
}


/**
 * IPC send network_msg to main_task
 */

void
ipc_send_network_msg_to_main_task(network_msg_t const * const network_msg, ipc_t const * const ipc)
{
    ipc_to_main_msg_t main_msg = {
        .typ = IPC_TO_HOME_TYP_NETWORK_MSG,
        .u = {
            .network_msg = *network_msg,
        },
    };
    if (xQueueSendToBack(ipc->to_home_q, &main_msg, 0) != pdPASS) {
        ESP_LOGW(TAG, "to_home_q full");
    }
    vTaskDelay(1);  // give others a chance to catch up
}

/**
 * IPC send network_msg to pool_task
 **/

void
ipc_send_network_msg_to_pool_task(network_msg_t const * const network_msg, ipc_t const * const ipc)
{
    if (xQueueSendToBack(ipc->to_pool_q, network_msg, 0) != pdPASS) {
        ESP_LOGW(TAG, "to_pool_q full");
    }
}

} // namespace opnpool
} // namespace esphome