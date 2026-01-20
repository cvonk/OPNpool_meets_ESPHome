#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cJSON.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"
#include "network.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct ipc_t;

struct poolstate_time_t {
    uint8_t  hour;
    uint8_t  minute;
};

struct poolstate_date_t {
    uint8_t   day;
    uint8_t   month;
    uint16_t  year;
};

struct poolstate_tod_t {
    poolstate_date_t  date;
    poolstate_time_t  time;
};

struct poolstate_version_t {
    uint8_t  major;
    uint8_t  minor;
};

struct poolstate_system_t {
    poolstate_tod_t      tod;
    poolstate_version_t  version;
};

enum class poolstate_elem_system_typ_t : uint8_t {
    TIME         = 0,
    CTRL_VERSION = 1,
    IF_VERSION   = 2
};

enum class poolstate_thermo_typ_t : uint8_t {
    POOL = 0,
    SPA  = 1
};

struct poolstate_thermo_t {
    uint8_t              temp_in_f;
    uint8_t              set_point_in_f;
    network_heat_src_t   heat_src;
    bool                 heating;
};

enum class poolstate_elem_thermos_typ_t : uint8_t {
    TEMP      = 0,
    SET_POINT = 1,
    HEAT_SRC  = 2,
    HEATING   = 3
};

enum class poolstate_elem_sched_typ_t : uint8_t {
    START = 0,
    STOP  = 1
};

struct poolstate_sched_t {
    bool      active;
    uint16_t  start;
    uint16_t  stop;
};

enum class poolstate_temp_typ_t : uint8_t {
    AIR   = 0,
    WATER = 1
};

struct poolstate_temp_t {
    uint8_t temp;
};

enum class PoolstateElemTempTyp : uint8_t {
    TEMP = 0
};

struct poolstate_modes_t {
    bool  is_set[enum_count<network_pool_mode_bits_t>()];  // IntelliSense flags this incorrectly
};

struct poolstate_circuits_t {
    bool  active[enum_count<network_pool_circuit_t>()];  // IntelliSense flags this incorrectly
    bool  delay[enum_count<network_pool_circuit_t>()];   // IntelliSense flags this incorrectly
};

enum class poolstate_elem_circuits_typ_t : uint8_t {
    ACTIVE = 0,
    DELAY = 1
};


struct poolstate_pump_t {
    poolstate_time_t     time;
    network_pump_mode_t  mode;
    bool                 running;
    network_pump_state_t state;
    uint16_t             power;
    uint16_t             flow;
    uint16_t             speed;
    uint16_t             level;
    uint8_t              error;
    uint8_t              timer;
};

#if 0
enum class poolstate_elem_pump_typ_t : uint8_t {
    TIME = 0,
    MODE = 1,
    RUNNING = 2,
    STATE = 3,
    POWER = 4,
    FLOW = 5,
    SPEED = 6,
    LEVEL = 7,
    ERROR = 8,
    TIMER = 9
};
#endif

enum class poolstate_chlor_status_t : uint8_t {
    OK         = 0,
    LOW_FLOW   = 1,
    LOW_SALT   = 2,
    HIGH_SALT  = 3,
    COLD       = 4,
    CLEAN_CELL = 5,
    OTHER      = 6
};

struct poolstate_chlor_t {
    char                      name[sizeof(network_msg_chlor_name_str_t) + 1];
    uint8_t                   level;
    uint16_t                  salt;
    poolstate_chlor_status_t  status;
};

#if 0
enum class poolstate_elem_chlor_typ_t : uint8_t {
    NAME   = 0,
    LEVEL  = 1,
    SALT   = 2,
    STATUS = 3
};
#endif

/**
 * @brief Complete pool state structure for the pool automation system.
 *
 * This structure contains the full state of the pool system, including:
 * - System information (date, time, firmware version)
 * - Temperature readings (air, water)
 * - Thermostat states (pool, spa)
 * - Schedules for each circuit
 * - Mode bitsets
 * - Circuit states (active, delay)
 * - Pump status (mode, running, power, etc.)
 * - Chlorinator status (name, level, salt, status)
 *
 * @var system   System information (date, time, firmware version)
 * @var temps    Array of temperature readings (e.g., air, water)
 * @var thermos  Array of thermostat states (pool, spa)
 * @var scheds   Array of schedules for each circuit
 * @var modes    Mode bitsets (which pool modes are active)
 * @var circuits Circuit states (active/delay for each circuit)
 * @var pump     Pump status (mode, running, power, speed, etc.)
 * @var chlor    Chlorinator status (name, level, salt, status)
 */
struct poolstate_t {
    bool                  valid;
    poolstate_system_t    system;
    poolstate_temp_t      temps[enum_count<poolstate_temp_typ_t>()];
    poolstate_thermo_t    thermos[enum_count<poolstate_thermo_typ_t>()];
    poolstate_sched_t     scheds[enum_count<network_pool_circuit_t>()];
    poolstate_modes_t     modes;
    poolstate_circuits_t  circuits;
    poolstate_pump_t      pump;
    poolstate_chlor_t     chlor;
};

enum class poolstate_elem_typ_t : uint8_t {
    SYSTEM   = 0x00,
    TEMP     = 0x01,
    THERMO   = 0x02,
    SCHED    = 0x03,
    CIRCUITS = 0x04,
    PUMP     = 0x05,
    CHLOR    = 0x06,
    MODES    = 0x07,
    ALL      = 0x08
};

#if 0
using poolstate_get_value_t = char *;

struct poolstate_get_params_t {
    poolstate_elem_typ_t elem_typ;
    uint8_t              elem_sub_typ;
    uint8_t const        idx;
};
#endif

/**
 * @brief Class representing the current state of the pool system
 **/    

     // forward declaration of OpnPool class
class OpnPool;

class PoolState {

    public:
        PoolState(OpnPool * const parent) : parent_{parent} {}
        ~PoolState() {}

        void set(poolstate_t const * const state) {
            memcpy(&last_poolstate_, state, sizeof(poolstate_t));
            last_poolstate_.valid = true;
        }
        esp_err_t get(poolstate_t * const state) {
            if (!last_poolstate_.valid) {
                return ESP_ERR_INVALID_STATE;
            }
            memcpy(state, &last_poolstate_, sizeof(poolstate_t));
            return ESP_OK;
        }

        bool has_changed(poolstate_t const * const state) {
            if (!last_poolstate_.valid) {  // redundant check
                return true;
            }   
            return memcmp(&last_poolstate_, state, sizeof(poolstate_t)) != 0;
        }


    private:
        OpnPool * const parent_;
        poolstate_t last_poolstate_ = { .valid = false };
        
};

}  // namespace opnpool
}  // namespace esphome