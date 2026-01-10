#ifndef __cplusplus
# error "This header requires C++ compilation"
#endif

#pragma once

#include <sdkconfig.h>
#include <esp_system.h>

namespace esphome {
namespace opnpool {

void pool_task(void * ipc_void);

}  // namespace opnpool
}  // namespace esphome