#pragma once
#ifndef __cplusplus
# error "This header requires C++ compilation"
#endif

#include <esp_system.h>
#include <cJSON.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"
#include "network.h"
#include "ipc.h"
#include "opnpool.h"
#include "network_msg.h"

namespace esphome {
namespace opnpool {

/**
 * poolstate_system_t
 **/

typedef struct poolstate_time_t {
    uint8_t hour;
    uint8_t minute;
} poolstate_time_t;

typedef struct poolstate_date_t {
    uint8_t day;
    uint8_t month;
    uint8_t year;
} poolstate_date_t;

typedef struct poolstate_tod_t {
    poolstate_date_t  date;
    poolstate_time_t  time;
} poolstate_tod_t;

typedef struct poolstate_version_t {
    uint8_t major;
    uint8_t minor;
} poolstate_version_t;

typedef struct poolstate_system_t {
    poolstate_tod_t     tod;
    poolstate_version_t version;
} poolstate_system_t;

enum class poolstate_elem_system_typ_t : uint8_t {
    TIME = 0,
    CTRL_VERSION = 1,
    IF_VERSION = 2,
};

/**
 * poolstate_thermo_typ_t
 **/

enum class poolstate_thermo_typ_t : uint8_t {
    POOL = 0,
    SPA = 1
};

constexpr size_t poolstate_thermo_typ_count() {
    return magic_enum::enum_count<poolstate_thermo_typ_t>();
}

inline const char * poolstate_str_thermo_str(poolstate_thermo_typ_t const thermostat_id) {
    auto name = magic_enum::enum_name(thermostat_id);
    return name.data();
}

inline int poolstate_str_thermo_nr(char const * const thermostat_str) {
    auto value = magic_enum::enum_cast<poolstate_thermo_typ_t>(thermostat_str);
    return value.has_value() ? static_cast<int>(*value) : -1;
}

typedef struct poolstate_thermo_t {
    uint8_t  temp;
    uint8_t  set_point;
    uint8_t  heat_src;
    bool     heating;
} poolstate_thermo_t;

enum class poolstate_elem_thermos_typ_t : uint8_t {
    TEMP = 0,
    SET_POINT = 1,
    HEAT_SRC = 2,
    HEATING = 3
};

/**
 * poolstate_sched_t
 */

enum class poolstate_elem_sched_typ_t : uint8_t {
    START = 0,
    STOP = 1
};

typedef struct poolstate_sched_t {
    bool      active;
    uint16_t  start;
    uint16_t  stop;
} poolstate_sched_t;

/**
 * poolstate_temp_t
 **/

enum class poolstate_temp_typ_t : uint8_t {
    AIR = 0,
    SOLAR = 1
};

constexpr size_t poolstate_temp_typ_count() {
    return magic_enum::enum_count<poolstate_temp_typ_t>();
}

inline const char * poolstate_str_temp_str(poolstate_temp_typ_t const temp_id) {
    auto name = magic_enum::enum_name(temp_id);
    return name.data();
}

inline int poolstate_str_temp_nr(char const * const temp_str) {
    auto value = magic_enum::enum_cast<poolstate_temp_typ_t>(temp_str);
    return value.has_value() ? static_cast<int>(*value) : -1;
}

typedef struct poolstate_temp_t {
    uint8_t temp;
} poolstate_temp_t;

enum class PoolstateElemTempTyp : uint8_t {
    TEMP = 0
};

/**
 * poolstate_modes_t
 **/

typedef struct poolstate_modes_t {
    bool  set[network_mode_count()];
} poolstate_modes_t;

enum class poolstate_elem_modes_typ_t : uint8_t {
    SERVICE = 0,
    TEMP_INC = 1,
    FREEZE_PROT = 2,
    TIMEOUT = 3
};

/**
 * poolstate_circuits_t
 **/

typedef struct poolstate_circuits_t {
    bool     active[network_circuit_count()];
    bool     delay[network_circuit_count()];
} poolstate_circuits_t;

enum class poolstate_elem_circuits_typ_t : uint8_t {
    ACTIVE = 0,
    DELAY = 1
};

/**
 * poolstate_pump_t
 **/

typedef struct {
    poolstate_time_t time;
    uint8_t  mode;
    bool     running;
    uint8_t  state;
    uint16_t pwr;
    uint16_t gpm;
    uint16_t rpm;
    uint16_t pct;
    uint8_t  err;
    uint8_t  timer;
} poolstate_pump_t;

enum class poolstate_elem_pump_typ_t : uint8_t {
    TIME = 0,
    MODE = 1,
    RUNNING = 2,
    STATE = 3,
    PWR = 4,
    GPM = 5,
    RPM = 6,
    PCT = 7,
    ERR = 8,
    TIMER = 9
};

/**
 * poolstate_chlor_t
 **/

enum class poolstate_chlor_status_t : uint8_t {
    OK = 0,
    LOW_FLOW = 1,
    LOW_SALT = 2,
    HIGH_SALT = 3,
    COLD = 4,
    CLEAN_CELL = 5,
    OTHER = 6
};

inline const char * poolstate_str_chlor_status_str(poolstate_chlor_status_t const chlor_state_id) {
    auto name = magic_enum::enum_name(chlor_state_id);
    return name.data();
}

inline int poolstate_str_chlor_status_nr(char const * const chlor_status_str) {
    auto value = magic_enum::enum_cast<poolstate_chlor_status_t>(chlor_status_str);
    return value.has_value() ? static_cast<int>(*value) : -1;
}

typedef struct poolstate_chlor_t {
    char                   name[sizeof(network_msg_chlor_name_str_t) + 1];
    uint8_t                pct;
    uint16_t               salt;
    poolstate_chlor_status_t    status;
} poolstate_chlor_t;

enum class poolstate_elem_chlor_typ_t : uint8_t {
    NAME = 0,
    PCT = 1,
    SALT = 2,
    STATUS = 3
};

/**
 * all together now
 **/

typedef struct poolstate_t {
    poolstate_system_t    system;
    poolstate_temp_t      temps[poolstate_temp_typ_count()];
    poolstate_thermo_t    thermos[poolstate_thermo_typ_count()];
    poolstate_sched_t     scheds[network_circuit_count()];
    poolstate_modes_t     modes;
    poolstate_circuits_t  circuits;
    poolstate_pump_t      pump;
    poolstate_chlor_t     chlor;
} poolstate_t;

enum class poolstate_elem_typ_t : uint8_t {
    SYSTEM = 0x00,
    TEMP = 0x01,
    THERMO = 0x02,
    SCHED = 0x03,
    CIRCUITS = 0x04,
    PUMP = 0x05,
    CHLOR = 0x06,
    MODES = 0x07,
    ALL = 0x08
};

typedef char * poolstate_get_value_t;
typedef struct poolstate_get_params_t {
    poolstate_elem_typ_t elem_typ;
    uint8_t          elem_sub_typ;
    uint8_t const    idx;
} poolstate_get_params_t;

class OpnPool;

class OpnPoolState {

    public:
        OpnPoolState(OpnPool *parent);
        bool is_valid() const { return protected_.valid; }
        
        void set(poolstate_t const * const state);
        esp_err_t get(poolstate_t * const state);
        esp_err_t update(network_msg_t const * const msg);
        esp_err_t get_value(poolstate_get_params_t const * const params, poolstate_get_value_t * value);
        const char * to_json(poolstate_elem_typ_t const typ);

        esp_err_t rx_update(network_msg_t const * const msg);
        esp_err_t get_poolstate_value(poolstate_t const * const state, poolstate_get_params_t const * const params, poolstate_get_value_t * value);

    private:
        typedef struct poolstate_prot_t {
            SemaphoreHandle_t xMutex;
            poolstate_t * state;
            bool valid;
        } poolstate_prot_t;

        OpnPool * parent_;
        poolstate_prot_t protected_;
        
        void rx_ctrl_time(cJSON * const dbg, network_msg_ctrl_time_t const * const msg, poolstate_t * const state);
        void rx_ctrl_heat_resp(cJSON * const dbg, network_msg_ctrl_heat_resp_t const * const msg, poolstate_t * const state);
        void rx_ctrl_heat_set(cJSON * const dbg, network_msg_ctrl_heat_set_t const * const msg, poolstate_t * const state);
        void rx_ctrl_circuit_set(cJSON * const dbg, network_msg_ctrl_circuit_set_t const * const msg, poolstate_t * const state);
        void rx_ctrl_sched_resp(cJSON * const dbg, network_msg_ctrl_sched_resp_t const * const msg, poolstate_t * const state);
        void rx_ctrl_state(cJSON * const dbg, network_msg_ctrl_state_bcast_t const * const msg, poolstate_t * state);
        void rx_ctrl_version_resp(cJSON * const dbg, network_msg_ctrl_version_resp_t const * const msg, poolstate_t * state);
        void rx_pump_reg_set(cJSON * const dbg, network_msg_pump_reg_set_t const * const msg);
        void rx_pump_reg_set_resp(cJSON * const dbg, network_msg_pump_reg_resp_t const * const msg);
        void rx_pump_ctrl(cJSON * const dbg, network_msg_pump_ctrl_t const * const msg);
        void rx_pump_mode(cJSON * const dbg, network_msg_pump_mode_t const * const msg, poolstate_t * const state);
        void rx_pump_run(cJSON * const dbg, network_msg_pump_run_t const * const msg, poolstate_t * const state);
        void rx_pump_status(cJSON * const dbg, network_msg_pump_status_resp_t const * const msg, poolstate_t * const state);
        void rx_ctrl_set_ack(cJSON * const dbg, network_msg_ctrl_set_ack_t const * const msg);
        void rx_chlor_name_resp(cJSON * const dbg, network_msg_chlor_name_resp_t const * const msg, poolstate_t * const state);
        void rx_chlor_level_set(cJSON * const dbg, network_msg_chlor_level_set_t const * const msg, poolstate_t * const state);
        void rx_chlor_level_set_resp(cJSON * const dbg, network_msg_chlor_level_resp_t const * const msg, poolstate_t * const state);
};

/* helpers from poolstate_log.cpp */
void opnpoolstate_log_add_tod(cJSON * const obj, char const * const key, poolstate_tod_t const * const tod);
void opnpoolstate_log_add_system(cJSON * const obj, char const * const key, poolstate_t const * const state);
void opnpoolstate_log_add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos, bool const showTemp, bool showSp, bool const showSrc, bool const showHeating);
void opnpoolstate_log_add_sched(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds, bool const showSched);
void opnpoolstate_log_add_state(cJSON * const obj, char const * const key, poolstate_t const * const state);
void opnpoolstate_log_add_pump_program(cJSON * const obj, char const * const key, uint16_t const value);
void opnpoolstate_log_add_pump_ctrl(cJSON * const obj, char const * const key, uint8_t const ctrl);
void opnpoolstate_log_add_pump_mode(cJSON * const obj, char const * const key, uint8_t const mode);
void opnpoolstate_log_add_pump_running(cJSON * const obj, char const * const key, bool const running);
void opnpoolstate_log_add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state);
void opnpoolstate_log_add_chlor_resp(cJSON * const obj, char const * const key, poolstate_chlor_t const * const chlor);
void opnpoolstate_log_add_version(cJSON * const obj, char const * const key, poolstate_version_t const * const version);
char const * opnpoolstate_log_state(poolstate_t const * const state, poolstate_elem_typ_t const typ);

}  // namespace opnpool
}  // namespace esphome