#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <vector>
#include <string>
#include <esphome/core/component.h>
#include "enum_helpers.h"

namespace esphome {
namespace opnpool {

    // forward declarations
struct ipc_t;
struct rs485_pins_t;
struct poolstate_t;
struct pending_switch_t;
struct pending_climate_t;
class PoolState; 
class OpnPoolClimate;
class OpnPoolSwitch;
class OpnPoolSensor;
class OpnPoolBinarySensor;
class OpnPoolTextSensor;

// IDs for array indexing (will be OVERWRITTEN by __init__.py for consistency with its CONF_*)

enum class ClimateId : uint8_t {
    POOL_CLIMATE = 0,
    SPA_CLIMATE = 1,
};
enum class SwitchId : uint8_t {
    SPA = 0,
    AUX1 = 1,
    AUX2 = 2,
    AUX3 = 3,
    FEATURE1 = 4,
    POOL = 5,
    FEATURE2 = 6,
    FEATURE3 = 7,
    FEATURE4 = 8,
};
enum class SensorId : uint8_t {
    AIR_TEMPERATURE = 0,
    WATER_TEMPERATURE = 1,
    PUMP_POWER = 2,
    PUMP_FLOW = 3,
    PUMP_SPEED = 4,
    CHLORINATOR_LEVEL = 5,
    CHLORINATOR_SALT = 6,
    PUMP_ERROR = 7
};
enum class BinarySensorId : uint8_t {
    PUMP_RUNNING = 0,
    MODE_SERVICE = 1,
    MODE_TEMPERATURE_INC = 2,
    MODE_FREEZE_PROTECTION = 3,
    MODE_TIMEOUT = 4,
};
enum class TextSensorId : uint8_t {
    POOL_SCHED = 0,
    SPA_SCHED = 1,
    PUMP_MODE = 2,
    PUMP_STATE = 3,
    CHLORINATOR_NAME = 4,
    CHLORINATOR_STATUS = 5,
    SYSTEM_TIME = 6,
    CONTROLLER_FIRMWARE = 7,
    INTERFACE_FIRMWARE = 8
};


    // main component

class OpnPool : public Component {

  public:
    
    void setup() override;
    void loop() override;
    void dump_config() override;

        // RS485 configuration
    void set_rs485_pins(uint8_t rx_pin, uint8_t tx_pin, uint8_t flow_control_pin);

        // climate setters        
    void set_pool_climate(OpnPoolClimate * const climate);
    void set_spa_climate(OpnPoolClimate * const climate);

        // switch setters
    void set_pool_switch(OpnPoolSwitch * const sw);
    void set_spa_switch(OpnPoolSwitch * const sw);
    void set_aux1_switch(OpnPoolSwitch * const sw);
    void set_aux2_switch(OpnPoolSwitch * const sw);
    void set_aux3_switch(OpnPoolSwitch * const sw);
    void set_feature1_switch(OpnPoolSwitch * const sw);
    void set_feature2_switch(OpnPoolSwitch * const sw);
    void set_feature3_switch(OpnPoolSwitch * const sw);
    void set_feature4_switch(OpnPoolSwitch * const sw);

        // sensor setters
    void set_air_temperature_sensor(OpnPoolSensor * const s);
    void set_water_temperature_sensor(OpnPoolSensor * const s);
    void set_pump_power_sensor(OpnPoolSensor * const s);
    void set_pump_flow_sensor(OpnPoolSensor * const s);
    void set_pump_speed_sensor(OpnPoolSensor * const s);
    void set_pump_error_sensor(OpnPoolSensor * const s);
    void set_chlorinator_level_sensor(OpnPoolSensor * const s);
    void set_chlorinator_salt_sensor(OpnPoolSensor * const s);

        // binary sensor setters
    void set_pump_running_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_service_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_timeout_binary_sensor(OpnPoolBinarySensor * const bs);

        // text sensor setters
    void set_pool_sched_text_sensor(OpnPoolTextSensor * const ts);
    void set_spa_sched_text_sensor(OpnPoolTextSensor * const ts);
    void set_pump_mode_text_sensor(OpnPoolTextSensor * const ts);
    void set_pump_state_text_sensor(OpnPoolTextSensor * const ts);
    void set_chlorinator_name_text_sensor(OpnPoolTextSensor * const ts);
    void set_chlorinator_status_text_sensor(OpnPoolTextSensor * const ts);
    void set_system_time_text_sensor(OpnPoolTextSensor * const ts);
    void set_controller_firmware_text_sensor(OpnPoolTextSensor * const ts);
    void set_interface_firmware_text_sensor(OpnPoolTextSensor * const ts);

        // update sensors
    void update_climates(poolstate_t const * const state);
    void update_switches(poolstate_t const * const state);
    void update_text_sensors(poolstate_t const * const state);
    void update_analog_sensors(poolstate_t const * const state);
    void update_binary_sensors(poolstate_t const * const state);
    void update_all(poolstate_t const * const state);

    ipc_t * get_ipc() { return ipc_; }
    PoolState * get_opnpool_state() { return poolState_; }
    OpnPoolSwitch * get_switch(uint8_t id) { return this->switches_[id]; }

  protected:

    ipc_t * ipc_{nullptr};  // interprocess communication structure and RS485-pins
    PoolState * poolState_{nullptr};

    void service_requests_from_pool(ipc_t const * const ipc);

        // sub classes
    OpnPoolClimate *climates_[enum_count<ClimateId>()]{nullptr};
    OpnPoolSwitch *switches_[enum_count<SwitchId>()]{nullptr};    
    OpnPoolSensor *sensors_[enum_count<SensorId>()]{nullptr};
    OpnPoolBinarySensor *binary_sensors_[enum_count<BinarySensorId>()]{nullptr};
    OpnPoolTextSensor *text_sensors_[enum_count<TextSensorId>()]{nullptr};
};

} // namespace opnpool
} // namespace esphome