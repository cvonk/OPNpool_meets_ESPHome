#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // function prototypes for pool_task.cpp
void pool_task(void * ipc_void);

}  // namespace opnpool
}  // namespace esphome