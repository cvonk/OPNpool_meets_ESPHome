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
enum network_msg_dev_id_t;

    // namespace-scope free functions from poolstate_log.cpp

namespace poolstate_rx {
namespace poolstate_rx_log {

    // JSON key names
inline constexpr char const * const KEY_TIME     = "time";
inline constexpr char const * const KEY_DATE     = "date";
inline constexpr char const * const KEY_FIRMWARE = "firmware";
inline constexpr char const * const KEY_TOD      = "tod";
inline constexpr char const * const KEY_TEMP     = "temp";
inline constexpr char const * const KEY_SP       = "sp";
inline constexpr char const * const KEY_SRC      = "src";
inline constexpr char const * const KEY_HEATING  = "heating";
inline constexpr char const * const KEY_START    = "start";
inline constexpr char const * const KEY_STOP     = "stop";
inline constexpr char const * const KEY_ACTIVE   = "active";
inline constexpr char const * const KEY_DELAY    = "delay";
inline constexpr char const * const KEY_SYSTEM   = "system";
inline constexpr char const * const KEY_TEMPS    = "temps";
inline constexpr char const * const KEY_THERMOS  = "thermos";
inline constexpr char const * const KEY_PUMP     = "pump";
inline constexpr char const * const KEY_CHLOR    = "chlor";
inline constexpr char const * const KEY_CIRCUITS = "circuits";
inline constexpr char const * const KEY_SCHEDS   = "scheds";
inline constexpr char const * const KEY_MODES    = "modes";
inline constexpr char const * const KEY_NAME     = "name";
inline constexpr char const * const KEY_LEVEL    = "level";
inline constexpr char const * const KEY_SALT     = "salt";
inline constexpr char const * const KEY_STATUS   = "status";
inline constexpr char const * const KEY_MODE     = "mode";
inline constexpr char const * const KEY_RUNNING  = "running";
inline constexpr char const * const KEY_STATE    = "state";
inline constexpr char const * const KEY_POWER    = "power";
inline constexpr char const * const KEY_SPEED    = "speed";
inline constexpr char const * const KEY_FLOW     = "flow";
inline constexpr char const * const KEY_ERROR    = "error";
inline constexpr char const * const KEY_TIMER    = "timer";
inline constexpr char const * const KEY_RESP     = "resp";
inline constexpr char const * const KEY_CTRL     = "ctrl";

void add_time_and_date(cJSON * const obj, char const * const key, poolstate_tod_t const * const tod);
void add_version(cJSON * const obj, char const * const key, poolstate_version_t const * const version);
void add_thermos(cJSON * const obj, char const * const key, poolstate_thermo_t const * thermos, bool const showTemp, bool showSp, bool const showHeating);
void add_scheds(cJSON * const obj, char const * const key, poolstate_sched_t const * scheds);
void add_state(cJSON * const obj, char const * const key, poolstate_t const * const state);
void add_pump_program(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, uint16_t const value);
void add_pump_ctrl(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, network_pump_ctrl_t const ctrl);
void add_pump_mode(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, network_pump_mode_t const mode);
void add_pump_running(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, bool const running);
void add_pump(cJSON * const obj, char const * const key, network_msg_dev_id_t const dev_id, poolstate_pump_t const * const pumps);
void add_chlor_resp(cJSON * const obj, char const * const key, poolstate_chlor_t const * const chlor);

}  // namespace poolstate_rx_log
}  // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome