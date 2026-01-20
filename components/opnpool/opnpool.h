#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>

#include "enum_helpers.h"
#include "opnpool_ids.h"

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
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
    OpnPoolClimate *climates_[enum_count<climate_id_t>()]{nullptr};
    OpnPoolSwitch *switches_[enum_count<switch_id_t>()]{nullptr};    
    OpnPoolSensor *sensors_[enum_count<sensor_id_t>()]{nullptr};
    OpnPoolBinarySensor *binary_sensors_[enum_count<binary_sensor_id_t>()]{nullptr};
    OpnPoolTextSensor *text_sensors_[enum_count<text_sensor_id_t>()]{nullptr};
};

} // namespace opnpool
} // namespace esphome