#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <vector>
#include <string>
#include <esphome/core/component.h>

namespace esphome {
namespace opnpool {

    // forward declarations
struct ipc_t;
struct rs485_pins_t;
struct poolstate_t;
struct pending_switch_t;
struct pending_climate_t;
class OpnPoolState; 
class OpnPoolClimate;
class OpnPoolSwitch;
class OpnPoolSensor;
class OpnPoolBinarySensor;
class OpnPoolTextSensor;

// IDs for array indexing
enum class ClimateId : uint8_t {  // MUST MATCH CONF_CLIMATES in __init__.py
    POOL = 0,
    SPA,
    COUNT
};
enum class SwitchId : uint8_t {  // MUST MATCH CONF_SWITCHES in __init__.py
    POOL = 0,
    SPA,
    AUX1,
    AUX2,
    AUX3,
    FEATURE1,
    FEATURE2,
    FEATURE3,
    FEATURE4,
    COUNT
};
enum class SensorId : uint8_t {  // MUST MATCH CONF_ANALOG_SENSORS in __init__.py
    AIR_TEMP = 0,
    WATER_TEMP,
    PUMP_POWER,
    PUMP_FLOW,
    PUMP_SPEED,
    PUMP_ERROR,
    CHLOR_LEVEL,
    CHLOR_SALT,
    COUNT
};
enum class BinarySensorId : uint8_t {  // MUST MATCH CONF_BINARY_SENSORS in __init__.py
    PUMP_RUNNING = 0,
    MODE_SERVICE,
    MODE_TEMP_INC,
    MODE_FREEZE,
    MODE_TIMEOUT,
    COUNT
};
enum class TextSensorId : uint8_t {  // MUST MATCH CONF_TEXT_SENSORS in __init__.py 
    POOL_SCHED = 0,
    SPA_SCHED,
    PUMP_MODE,
    PUMP_STATE,
    CHLOR_NAME,
    CHLOR_STATUS,
    SYSTEM_TIME,
    CONTROLLER_FW,
    INTERFACE_FW,
    COUNT
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
    void set_controller_firmware_version_text_sensor(OpnPoolTextSensor * const ts);
    void set_interface_firmware_version_text_sensor(OpnPoolTextSensor * const ts);

    //void write_packet(uint8_t command, const std::vector<uint8_t> &payload);

        // update sensors
    void update_climates(poolstate_t const * const state);
    void update_switches(poolstate_t const * const state);
    void update_text_sensors(poolstate_t const * const state);
    void update_analog_sensors(poolstate_t const * const state);
    void update_binary_sensors(poolstate_t const * const state);
    void update_all(poolstate_t const * const state);

    ipc_t * get_ipc() { return ipc_; }
    OpnPoolState * get_opnpool_state() { return opnPoolState_; }
    OpnPoolSwitch * get_switch(uint8_t id) { return this->switches_[id]; }

  protected:

    ipc_t * ipc_{};  // interprocess communication structure and RS485-pins
    OpnPoolState * opnPoolState_{nullptr};

    void service_requests_from_pool(ipc_t const * const ipc);

        // sub classes
    OpnPoolClimate *climates_[static_cast<uint8_t>(ClimateId::COUNT)]{nullptr};
    OpnPoolSwitch *switches_[static_cast<uint8_t>(SwitchId::COUNT)]{nullptr};    
    OpnPoolSensor *sensors_[static_cast<uint8_t>(SensorId::COUNT)]{nullptr};
    OpnPoolBinarySensor *binary_sensors_[static_cast<uint8_t>(BinarySensorId::COUNT)]{nullptr};
    OpnPoolTextSensor *text_sensors_[static_cast<uint8_t>(TextSensorId::COUNT)]{nullptr};
};

} // namespace opnpool
} // namespace esphome