#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <cJSON.h>

#define MAGIC_ENUM_RANGE_MIN 0
#define MAGIC_ENUM_RANGE_MAX 256
#include "magic_enum.h"

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct poolstate_t;
struct poolstate_tod_t;
struct poolstate_version_t;
struct poolstate_thermo_t;
struct poolstate_sched_t;
struct poolstate_chlor_t;
enum network_pump_mode_t;
enum poolstate_elem_typ_t;

    // namespace-scope free functions from poolstate_log.cpp

namespace poolstate_rx {
namespace poolstate_rx_log {

void add_time_and_date(cJSON * const obj, char const * const key, poolstate_tod_t const * const tod);
void add_version(cJSON * const obj, char const * const key, poolstate_version_t const * const version);
void add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos, bool const showTemp, bool showSp, bool const showHeating);
void add_scheds(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds);
void add_state(cJSON * const obj, char const * const key, poolstate_t const * const state);
void add_pump_program(cJSON * const obj, char const * const key, uint16_t const value);
void add_pump_ctrl(cJSON * const obj, char const * const key,  network_pump_ctrl_t const ctrl);
void add_pump_mode(cJSON * const obj, char const * const key, network_pump_mode_t const mode);
void add_pump_running(cJSON * const obj, char const * const key, bool const running);
void add_pump(cJSON * const obj, char const * const key, poolstate_t const * const state);
void add_chlor_resp(cJSON * const obj, char const * const key, poolstate_chlor_t const * const chlor);

} // namespace poolstate_rx_log
} // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome