#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <vector>

namespace esphome {
namespace opnpool {

enum OpnPoolDebugLevel {
  DEBUG_LEVEL_NONE = 0,
  DEBUG_LEVEL_ERROR = 1,
  DEBUG_LEVEL_WARN = 2,
  DEBUG_LEVEL_INFO = 3,
  DEBUG_LEVEL_CONFIG = 4,
  DEBUG_LEVEL_DEBUG = 5,
  DEBUG_LEVEL_VERBOSE = 6,
  DEBUG_LEVEL_VERY_VERBOSE = 7
};  
class OpnPool; // Forward declaration for parent referencing

// climate entity

class OpnPoolClimate : public esphome::climate::Climate {
  public:
    esphome::climate::ClimateTraits traits() override;
    void control(const esphome::climate::ClimateCall &call) override;  
    void set_parent(OpnPool *parent) { this->parent_ = parent; }

  protected:
    OpnPool *parent_{nullptr};
};

// switch entity

class OpnPoolSwitch : public esphome::switch_::Switch {
 protected:
  void write_state(bool state) override { this->publish_state(state); }
};

class OpnPoolSensor : public esphome::sensor::Sensor {
 protected:
  // The base class sensor::Sensor already provides publish_state(float state)
};

class OpnPoolBinarySensor : public esphome::binary_sensor::BinarySensor {
 protected:
};

class OpnPoolTextSensor : public esphome::text_sensor::TextSensor {
 protected:
};

// main component

class OpnPool : public Component, public uart::UARTDevice {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;

    // climate setters
    void set_pool_heater(OpnPoolClimate *c) { pool_heater_ = c; if (c) c->set_parent(this); }
    void set_spa_heater(OpnPoolClimate *c) { spa_heater_ = c; if (c) c->set_parent(this); }

    // switch setters
    void set_pool_switch(switch_::Switch *s) { pool_sw_ = s; }
    void set_spa_switch(switch_::Switch *s) { spa_sw_ = s; }
    void set_aux1_switch(switch_::Switch *s) { aux1_sw_ = s; }
    void set_aux2_switch(switch_::Switch *s) { aux2_sw_ = s; }
    void set_feature1_switch(switch_::Switch *s) { feature1_sw_ = s; }
    void set_feature2_switch(switch_::Switch *s) { feature2_sw_ = s; }
    void set_feature3_switch(switch_::Switch *s) { feature3_sw_ = s; }
    void set_feature4_switch(switch_::Switch *s) { feature4_sw_ = s; }

    // analog sensor setters
    void set_air_temperature_sensor(sensor::Sensor *s) { air_temp_s_ = s; }
    void set_water_temperature_sensor(sensor::Sensor *s) { water_temp_s_ = s; }
    void set_pump_power_sensor(sensor::Sensor *s) { pump_power_s_ = s; }
    void set_pump_flow_sensor(sensor::Sensor *s) { pump_flow_s_ = s; }
    void set_pump_speed_sensor(sensor::Sensor *s) { pump_speed_s_ = s; }
    void set_chlorinator_level_sensor(sensor::Sensor *s) { chlor_level_s_ = s; }
    void set_chlorinator_salt_sensor(sensor::Sensor *s) { chlor_salt_s_ = s; }
    void set_pump_status_sensor(sensor::Sensor *s) { pump_status_s_ = s; }
    void set_pump_state_sensor(sensor::Sensor *s) { pump_state_s_ = s; }
    void set_pump_error_sensor(sensor::Sensor *s) { pump_error_s_ = s; }

    // text sensor setters
    void set_pool_sched_text_sensor(text_sensor::TextSensor *s) { pool_sched_ts_ = s; }
    void set_spa_sched_text_sensor(text_sensor::TextSensor *s) { spa_sched_ts_ = s; }
    void set_aux1_sched_text_sensor(text_sensor::TextSensor *s) { aux1_sched_ts_ = s; }
    void set_aux2_sched_text_sensor(text_sensor::TextSensor *s) { aux2_sched_ts_ = s; }
    void set_system_time_text_sensor(text_sensor::TextSensor *s) { system_time_ts_ = s; }
    void set_controller_firmware_version_text_sensor(text_sensor::TextSensor *s) { controller_fw_ts_ = s; }
    void set_interface_firmware_version_text_sensor(text_sensor::TextSensor *s) { interface_fw_ts_ = s; }
    void set_pump_mode_text_sensor(text_sensor::TextSensor *s) { pump_mode_ts_ = s; }
    void set_chlorinator_name_text_sensor(text_sensor::TextSensor *s) { chlor_name_ts_ = s; }
    void set_chlorinator_status_text_sensor(text_sensor::TextSensor *s) { chlor_status_ts_ = s; }

    // binary sensor setters
    void set_pump_running_binary_sensor(binary_sensor::BinarySensor *s) { pump_running_bs_ = s; }
    void set_mode_service_binary_sensor(binary_sensor::BinarySensor *s) { mode_service_bs_ = s; }
    void set_mode_temperature_inc_binary_sensor(binary_sensor::BinarySensor *s) { mode_temp_inc_bs_ = s; }
    void set_mode_freeze_protection_binary_sensor(binary_sensor::BinarySensor *s) { mode_freeze_bs_ = s; }
    void set_mode_timeout_binary_sensor(binary_sensor::BinarySensor *s) { mode_timeout_bs_ = s; }

    // debug level setters
    void set_datalink_debug(int level) { datalink_level_ = level; }
    void set_network_debug(int level) { network_level_ = level; }
    void set_pool_state_debug(int level) { pool_state_level_ = level; }
    void set_pool_task_debug(int level) { pool_task_level_ = level; }
    void set_mqtt_task_debug(int level) { mqtt_task_level_ = level; }
    void set_hass_task_debug(int level) { hass_task_level_ = level; }

    void write_packet(uint8_t command, const std::vector<uint8_t> &payload);

  protected:
    void parse_packet_(const std::vector<uint8_t> &data);

    std::vector<uint8_t> rx_buffer_;

    // debug levels
    int datalink_level_{ESPHOME_LOG_LEVEL_INFO};
    int network_level_{ESPHOME_LOG_LEVEL_INFO};
    int pool_state_level_{ESPHOME_LOG_LEVEL_INFO};
    int pool_task_level_{ESPHOME_LOG_LEVEL_INFO};
    int mqtt_task_level_{ESPHOME_LOG_LEVEL_INFO};
    int hass_task_level_{ESPHOME_LOG_LEVEL_INFO};    
    bool should_log_(int module_level, int check_level) { return module_level >= check_level; }    

    // member pointers
    OpnPoolClimate *pool_heater_{nullptr}, *spa_heater_{nullptr};
    switch_::Switch *pool_sw_{nullptr}, *spa_sw_{nullptr}, *aux1_sw_{nullptr}, *aux2_sw_{nullptr};
    switch_::Switch *feature1_sw_{nullptr}, *feature2_sw_{nullptr}, *feature3_sw_{nullptr}, *feature4_sw_{nullptr};

    sensor::Sensor *air_temp_s_{nullptr}, *water_temp_s_{nullptr}, *pump_power_s_{nullptr}, *pump_flow_s_{nullptr};
    sensor::Sensor *pump_speed_s_{nullptr}, *chlor_level_s_{nullptr}, *chlor_salt_s_{nullptr};
    sensor::Sensor *pump_status_s_{nullptr}, *pump_state_s_{nullptr}, *pump_error_s_{nullptr};

    binary_sensor::BinarySensor *pump_running_bs_{nullptr}, *mode_service_bs_{nullptr}, *mode_temp_inc_bs_{nullptr};
    binary_sensor::BinarySensor *mode_freeze_bs_{nullptr}, *mode_timeout_bs_{nullptr};

    text_sensor::TextSensor *pool_sched_ts_{nullptr}, *spa_sched_ts_{nullptr}, *aux1_sched_ts_{nullptr}, *aux2_sched_ts_{nullptr};
    text_sensor::TextSensor *system_time_ts_{nullptr}, *controller_fw_ts_{nullptr}, *interface_fw_ts_{nullptr};
    text_sensor::TextSensor *pump_mode_ts_{nullptr}, *chlor_name_ts_{nullptr}, *chlor_status_ts_{nullptr};
};

} // namespace opnpool
} // namespace esphome