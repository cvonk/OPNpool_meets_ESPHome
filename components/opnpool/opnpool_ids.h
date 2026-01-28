#pragma once

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
enum class poolstate_thermo_typ_t : uint8_t;
enum class network_pool_circuit_t : uint8_t;

    // IDs for array indexing (will be OVERWRITTEN by __init__.py for consistency with its CONF_*)

enum class climate_id_t : uint8_t {
    POOL_CLIMATE = 0,
    SPA_CLIMATE  = 1
};

enum class switch_id_t : uint8_t {
    SPA      = 0,
    AUX1     = 1,
    AUX2     = 2,
    AUX3     = 3,
    FEATURE1 = 4,
    POOL     = 5,
    FEATURE2 = 6,
    FEATURE3 = 7,
    FEATURE4 = 8
};
enum class sensor_id_t : uint8_t {
    AIR_TEMPERATURE   = 0,
    WATER_TEMPERATURE = 1,
    PUMP_POWER        = 2,
    PUMP_FLOW         = 3,
    PUMP_SPEED        = 4,
    CHLORINATOR_LEVEL = 5,
    CHLORINATOR_SALT  = 6,
    PUMP_ERROR        = 7
};
enum class binary_sensor_id_t : uint8_t {
    PUMP_RUNNING           = 0,
    MODE_SERVICE           = 1,
    MODE_TEMPERATURE_INC   = 2,
    MODE_FREEZE_PROTECTION = 3,
    MODE_TIMEOUT           = 4
};
enum class text_sensor_id_t : uint8_t {
    POOL_SCHED          = 0,
    SPA_SCHED           = 1,
    PUMP_MODE           = 2,
    PUMP_STATE          = 3,
    CHLORINATOR_NAME    = 4,
    CHLORINATOR_STATUS  = 5,
    SYSTEM_TIME         = 6,
    CONTROLLER_FIRMWARE = 7,
    INTERFACE_FIRMWARE  = 8
};    

    // function prototypes for opnpool_ids.cpp
[[nodiscard]] poolstate_thermo_typ_t climate_id_to_poolstate_thermo(climate_id_t const id);
[[nodiscard]] network_pool_circuit_t switch_id_to_network_circuit(switch_id_t const id);

} // namespace opnpool
} // namespace esphome