#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esphome/core/component.h>
#include <esphome/components/uart/uart.h>                    // esphome::uart::UARTDevice
#include <esphome/components/climate/climate.h>              // esphome::climate::Climate
#include <esphome/components/switch/switch.h>                // esphome::switch_::Switch
#include <esphome/components/sensor/sensor.h>                // esphome::sensor::Sensor
#include <esphome/components/binary_sensor/binary_sensor.h>  // esphome::binary_sensor::BinarySensor
#include <esphome/components/text_sensor/text_sensor.h>      // esphome::text_sensor::TextSensor
#include <vector>
#include <string>

#include "ipc.h"

    // help IntelliSense with ESPHome namespaces
#ifdef __INTELLISENSE__
namespace esphome {
    namespace sensor { class Sensor; }
    namespace binary_sensor { class BinarySensor; }
    namespace text_sensor { class TextSensor; }
    namespace switch_ { class Switch; }
    namespace climate { class Climate; }
    namespace uart { class UARTDevice; }
}
#endif

namespace esphome {
namespace opnpool {
  
  // forward declarations
class OpnPool;
class OpnPoolState; 

// climate entity
class OpnPoolClimate : public climate::Climate {  // ✅ Changed from esphome::climate::Climate

  public:
    climate::ClimateTraits traits() override;  // ✅ Changed from esphome::climate::ClimateTraits
    void control(const climate::ClimateCall &call) override;  // ✅ Changed from esphome::climate::ClimateCall
    void set_parent(OpnPool *parent) { this->parent_ = parent; }

  protected:
    OpnPool *parent_{nullptr};
};

// switch entity
class OpnPoolSwitch : public switch_::Switch {  // ✅ Changed from esphome::switch_::Switch
  public:
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    void set_circuit_id(uint8_t circuit_id) { this->circuit_id_ = circuit_id; }
  
  protected:
    void write_state(bool state) override;
    OpnPool *parent_{nullptr};
    uint8_t circuit_id_{0};
};

// sensor entities
class OpnPoolSensor : public sensor::Sensor {  // ✅ Changed from esphome::sensor::Sensor
  protected:
    // The base class sensor::Sensor already provides publish_state(float state)
};

class OpnPoolBinarySensor : public binary_sensor::BinarySensor {  // ✅ Changed
   protected:
};

class OpnPoolTextSensor : public text_sensor::TextSensor {  // ✅ Changed
   protected:
};

// main component
class OpnPool : public Component, public uart::UARTDevice {  // ✅ No change needed here

  public:
    OpnPool();
    void setup() override;
    void loop() override;
    void dump_config() override;

    // RS485 configuration
    void set_rs485_rx_pin(uint8_t pin) { ipc_.config.rs485_pins.rx_pin = pin; }
    void set_rs485_tx_pin(uint8_t pin) { ipc_.config.rs485_pins.tx_pin = pin; }
    void set_rs485_flow_control_pin(uint8_t pin) { ipc_.config.rs485_pins.flow_control_pin = pin; }
    void set_rs485_config(const rs485_pins_t &cfg) { ipc_.config.rs485_pins = cfg; }    
    const rs485_pins_t &get_rs485_config() const { return ipc_.config.rs485_pins; }

    // climate setters
    void set_pool_heater(OpnPoolClimate *c) { pool_heater_ = c; if (c) c->set_parent(this); }
    void set_spa_heater(OpnPoolClimate *c) { spa_heater_ = c; if (c) c->set_parent(this); }

    // switch setters - ✅ Keep these as switch_::Switch (correct)
    void set_pool_switch(switch_::Switch *s) { pool_sw_ = s; }
    void set_spa_switch(switch_::Switch *s) { spa_sw_ = s; }
    void set_aux1_switch(switch_::Switch *s) { aux1_sw_ = s; }
    void set_aux2_switch(switch_::Switch *s) { aux2_sw_ = s; }
    void set_feature1_switch(switch_::Switch *s) { feature1_sw_ = s; }
    void set_feature2_switch(switch_::Switch *s) { feature2_sw_ = s; }
    void set_feature3_switch(switch_::Switch *s) { feature3_sw_ = s; }
    void set_feature4_switch(switch_::Switch *s) { feature4_sw_ = s; }

    // analog sensor setters - ✅ Keep these as sensor::Sensor (correct)
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

    // text sensor setters - ✅ Keep these as text_sensor::TextSensor (correct)
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

    // binary sensor setters - ✅ Keep these as binary_sensor::BinarySensor (correct)
    void set_pump_running_binary_sensor(binary_sensor::BinarySensor *s) { pump_running_bs_ = s; }
    void set_mode_service_binary_sensor(binary_sensor::BinarySensor *s) { mode_service_bs_ = s; }
    void set_mode_temperature_inc_binary_sensor(binary_sensor::BinarySensor *s) { mode_temp_inc_bs_ = s; }
    void set_mode_freeze_protection_binary_sensor(binary_sensor::BinarySensor *s) { mode_freeze_bs_ = s; }
    void set_mode_timeout_binary_sensor(binary_sensor::BinarySensor *s) { mode_timeout_bs_ = s; }

    void write_packet(uint8_t command, const std::vector<uint8_t> &payload);
    void on_switch_command(uint8_t circuit, bool state);

  protected:

    ipc_t ipc_{};  // interprocess communication structure and RS485-pins

    OpnPoolState * opnPoolState_{nullptr};
    void service_requests_from_pool(ipc_t const * const ipc);

    void parse_packet_(const std::vector<uint8_t> &data);
    std::vector<uint8_t> rx_buffer_;

    // member pointers - ✅ Keep these consistent
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