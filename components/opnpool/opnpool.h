#pragma once
#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <vector>

namespace esphome {
namespace opnpool {

class OpnPool; // Forward declaration for parent referencing

// --- Climate Entity ---
class OpnPoolClimate : public climate::Climate {
 public:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  
  // FIX: Added set_parent
  void set_parent(OpnPool *parent) { this->parent_ = parent; }

 protected:
  // FIX: Added parent_ member
  OpnPool *parent_{nullptr};
};

// --- Switch Entity ---
class OpnPoolSwitch : public switch_::Switch {
 protected:
  void write_state(bool state) override { this->publish_state(state); }
};

// --- Main Component ---
class OpnPool : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Climate Setters
  void set_pool_heater(OpnPoolClimate *c) { 
    pool_heater_ = c; 
    c->set_parent(this); 
  }
  void set_spa_heater(OpnPoolClimate *c) { 
    spa_heater_ = c; 
    c->set_parent(this); 
  }

  // Switch Setters
  void set_pool_switch(switch_::Switch *s) { pool_sw_ = s; }
  void set_spa_switch(switch_::Switch *s) { spa_sw_ = s; }
  void set_aux1_switch(switch_::Switch *s) { aux1_sw_ = s; }
  void set_aux2_switch(switch_::Switch *s) { aux2_sw_ = s; }
  void set_flt1_switch(switch_::Switch *s) { flt1_sw_ = s; }
  void set_flt2_switch(switch_::Switch *s) { flt2_sw_ = s; }
  void set_flt3_switch(switch_::Switch *s) { flt3_sw_ = s; }
  void set_flt4_switch(switch_::Switch *s) { flt4_sw_ = s; }

  // Analog Sensor Setters
  void set_air_temperature_sensor(sensor::Sensor *s) { air_temp_s_ = s; }
  void set_water_temperature_sensor(sensor::Sensor *s) { water_temp_s_ = s; }
  void set_pump_power_sensor(sensor::Sensor *s) { pump_power_s_ = s; }
  void set_pump_flow_sensor(sensor::Sensor *s) { pump_flow_s_ = s; }
  void set_pump_speed_sensor(sensor::Sensor *s) { pump_speed_s_ = s; }
  void set_chlorinator_percentage_sensor(sensor::Sensor *s) { chlor_pct_s_ = s; }
  void set_chlorinator_salt_sensor(sensor::Sensor *s) { chlor_salt_s_ = s; }
  void set_pump_status_sensor(sensor::Sensor *s) { pump_status_s_ = s; }
  void set_pump_state_sensor(sensor::Sensor *s) { pump_state_s_ = s; }
  void set_pump_error_sensor(sensor::Sensor *s) { pump_error_s_ = s; }
  void set_chlorinator_status_sensor(sensor::Sensor *s) { chlor_status_s_ = s; }

  // Text Sensor Setters
  void set_pool_sched_text_sensor(text_sensor::TextSensor *s) { pool_sched_ts_ = s; }
  void set_spa_sched_text_sensor(text_sensor::TextSensor *s) { spa_sched_ts_ = s; }
  void set_aux1_sched_text_sensor(text_sensor::TextSensor *s) { aux1_sched_ts_ = s; }
  void set_aux2_sched_text_sensor(text_sensor::TextSensor *s) { aux2_sched_ts_ = s; }
  void set_system_time_text_sensor(text_sensor::TextSensor *s) { system_time_ts_ = s; }
  void set_controller_firmware_version_text_sensor(text_sensor::TextSensor *s) { controller_fw_ts_ = s; }
  void set_interface_firmware_version_text_sensor(text_sensor::TextSensor *s) { interface_fw_ts_ = s; }
  void set_pump_mode_text_sensor(text_sensor::TextSensor *s) { pump_mode_ts_ = s; }
  void set_chlorinator_name_text_sensor(text_sensor::TextSensor *s) { chlor_name_ts_ = s; }

  // Binary Sensor Setters
  void set_pump_running_binary_sensor(binary_sensor::BinarySensor *s) { pump_running_bs_ = s; }
  void set_mode_service_binary_sensor(binary_sensor::BinarySensor *s) { mode_service_bs_ = s; }
  void set_mode_temperature_inc_binary_sensor(binary_sensor::BinarySensor *s) { mode_temp_inc_bs_ = s; }
  void set_mode_freeze_protection_binary_sensor(binary_sensor::BinarySensor *s) { mode_freeze_bs_ = s; }
  void set_mode_timeout_binary_sensor(binary_sensor::BinarySensor *s) { mode_timeout_bs_ = s; }

  // Communication Methods
  void write_packet(uint8_t command, const std::vector<uint8_t> &payload);

 protected:
  void process_byte_(uint8_t byte);
  void parse_packet_(const std::vector<uint8_t> &data);

  // Member Pointers
  OpnPoolClimate *pool_heater_{nullptr}, *spa_heater_{nullptr};
  switch_::Switch *pool_sw_{nullptr}, *spa_sw_{nullptr}, *aux1_sw_{nullptr}, *aux2_sw_{nullptr};
  switch_::Switch *flt1_sw_{nullptr}, *flt2_sw_{nullptr}, *flt3_sw_{nullptr}, *flt4_sw_{nullptr};

  sensor::Sensor *air_temp_s_{nullptr}, *water_temp_s_{nullptr}, *pump_power_s_{nullptr}, *pump_flow_s_{nullptr};
  sensor::Sensor *pump_speed_s_{nullptr}, *chlor_pct_s_{nullptr}, *chlor_salt_s_{nullptr};
  sensor::Sensor *pump_status_s_{nullptr}, *pump_state_s_{nullptr}, *pump_error_s_{nullptr}, *chlor_status_s_{nullptr};

  text_sensor::TextSensor *pool_sched_ts_{nullptr}, *spa_sched_ts_{nullptr}, *aux1_sched_ts_{nullptr}, *aux2_sched_ts_{nullptr};
  text_sensor::TextSensor *system_time_ts_{nullptr}, *controller_fw_ts_{nullptr}, *interface_fw_ts_{nullptr};
  text_sensor::TextSensor *pump_mode_ts_{nullptr}, *chlor_name_ts_{nullptr};

  binary_sensor::BinarySensor *pump_running_bs_{nullptr}, *mode_service_bs_{nullptr}, *mode_temp_inc_bs_{nullptr};
  binary_sensor::BinarySensor *mode_freeze_bs_{nullptr}, *mode_timeout_bs_{nullptr};

  std::vector<uint8_t> rx_buffer_;
};

} // namespace opnpool
} // namespace esphome