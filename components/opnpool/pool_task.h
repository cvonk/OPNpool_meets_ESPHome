#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <sdkconfig.h>
#include <esp_system.h>

namespace esphome {
namespace opnpool {

void pool_task(void * ipc_void);

}  // namespace opnpool
}  // namespace esphome