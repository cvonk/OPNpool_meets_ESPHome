/**
 * @file poolstate.h
 * @brief Pool State data structures and Interface for OPNpool component
 *
 * @details
 * This header defines the data structures and interface for representing and managing the
 * real-time state of the pool system in the OPNpool component. It maintains a
 * comprehensive software model of the pool controller and all connected peripherals
 * (pump, chlorinator, circuits, sensors, etc.), enabling accurate monitoring and control.
 *
 * The pool state is continuously updated in response to incoming network messages,
 * ensuring that the software state always reflects the latest equipment status and
 * configuration. This layer provides the foundation for publishing sensor values, driving
 * automation logic, and seamless integration with ESPHome and Home Assistant entities. By
 * abstracting hardware details and protocol specifics, it supports modular, robust, and
 * maintainable pool automation.
 *
 * The design supports modular separation of protocol, network, and application logic, and
 * is intended for use in a single-threaded ESPHome environment. Forward declarations are
 * provided to avoid circular dependencies.
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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cJSON.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"
#include "network.h"
#include "network_msg.h"

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#endif

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct ipc_t;

struct poolstate_time_t {
    bool     valid;
    uint8_t  hour;
    uint8_t  minute;
};

struct poolstate_date_t {
    bool      valid;
    uint8_t   day;
    uint8_t   month;
    uint16_t  year;
};

struct poolstate_tod_t {
    poolstate_date_t  date;
    poolstate_time_t  time;
};

struct poolstate_version_t {
    bool     valid;
    uint8_t  major;
    uint8_t  minor;
};

struct poolstate_system_t {
    poolstate_tod_t      tod;
    poolstate_version_t  version;
};

enum class poolstate_thermo_typ_t : uint8_t {
    POOL = 0,
    SPA  = 1
};

struct poolstate_thermo_t {
    bool                 valid;          // 2BD: make more granular
    uint8_t              temp_in_f;      // from ctrl_state_bcast, ctrl_heat_resp
    uint8_t              set_point_in_f; // from ctrl_heat_resp, ctrl_heat_set
    network_heat_src_t   heat_src;       // from ctrl_state_bcast, ctrl_heat_resp, ctrl_heat_set
    bool                 heating;        // from ctrl_state_bcast
};

struct poolstate_sched_t {
    bool      valid;
    bool      active;
    uint16_t  start;
    uint16_t  stop;
};

enum class poolstate_temp_typ_t : uint8_t {
    AIR   = 0,
    WATER = 1
};

struct poolstate_bool_t {
    bool valid;
    bool value;
};

struct poolstate_uint8_t {
    bool    valid;
    uint8_t value;
};

struct poolstate_uint16_t {
    bool     valid;
    uint16_t value;
};

struct poolstate_circuit_t {
    poolstate_bool_t active;
    poolstate_bool_t delay;
};

struct poolstate_pump_mode_t {
    bool                valid;
    network_pump_mode_t value;
};

struct poolstate_pump_state_t {
    bool                 valid;
    network_pump_state_t value;
};

struct poolstate_pump_t {
    poolstate_time_t       time;     // from pump_status_resp
    poolstate_pump_mode_t  mode;     // from pump_status_resp, pump_mode
    poolstate_bool_t       running;  // from pump_status_resp, pump_run
    poolstate_pump_state_t state;    // from pump_status_resp
    poolstate_uint16_t     power;    // from pump_status_resp 
    poolstate_uint16_t     flow;     // from pump_status_resp
    poolstate_uint16_t     speed;    // from pump_status_resp
    poolstate_uint16_t     level;    // from pump_status_resp
    poolstate_uint8_t      error;    // from pump_status_resp
    poolstate_uint8_t      timer;    // from pump_status_resp
};

enum class poolstate_chlor_status_typ_t : uint8_t {
    OK         = 0,
    LOW_FLOW   = 1,
    LOW_SALT   = 2,
    HIGH_SALT  = 3,
    COLD       = 4,
    CLEAN_CELL = 5,
    OTHER      = 6
};

struct poolstate_chlor_status_t {
    bool                         valid;
    poolstate_chlor_status_typ_t value;  // from chlor_level_resp
};

struct poolstate_chlor_name_t {
    bool     valid;
    char     value[sizeof(network_msg_chlor_name_str_t) + 1];
};

struct poolstate_chlor_t {
    poolstate_chlor_name_t   name;    // from chlor_name_resp
    poolstate_uint8_t        level;   // from chlor_level_set
    poolstate_uint16_t       salt;    // from chlor_level_resp, chlor_name_resp
    poolstate_chlor_status_t status;  // from chlor_level_resp
};

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
    poolstate_system_t   system;
    poolstate_chlor_t    chlor;
    poolstate_pump_t     pump;  // 2BD: may need to be an array for >1 pumps
    poolstate_circuit_t  circuits[enum_count<network_pool_circuit_t>()];
    poolstate_bool_t     modes[enum_count<network_pool_mode_bits_t>()];
    poolstate_thermo_t   thermos[enum_count<poolstate_thermo_typ_t>()];
    poolstate_uint8_t    temps[enum_count<poolstate_temp_typ_t>()];
    poolstate_sched_t    scheds[enum_count<network_pool_circuit_t>()];
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


/**
 * @brief Class representing the current state of the pool system
 **/    

     // forward declaration of OpnPool class
class OpnPool;

class PoolState {

    public:
        PoolState(OpnPool * const parent) : parent_{parent} {
            memset(&last_, 0, sizeof(poolstate_t));
        }
        ~PoolState() {}

        void set(poolstate_t const * const state) {
            memcpy(&last_, state, sizeof(poolstate_t));
        }
        esp_err_t get(poolstate_t * const state) {
            memcpy(state, &last_, sizeof(poolstate_t));
            return ESP_OK;
        }

        bool has_changed(poolstate_t const * const state) {
            return memcmp(&last_, state, sizeof(poolstate_t)) != 0;
        }

    private:
        OpnPool * const parent_;
        poolstate_t last_ = {};  // sets .valid bools to false as well
};

}  // namespace opnpool
}  // namespace esphome