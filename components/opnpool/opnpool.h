#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <vector>
#include <string>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"                    // esphome::uart::UARTDevice
#include "esphome/components/climate/climate.h"              // esphome::climate::Climate
#include "esphome/components/switch/switch.h"                // esphome::switch_::Switch
#include "esphome/components/sensor/sensor.h"                // esphome::sensor::Sensor
#include "esphome/components/binary_sensor/binary_sensor.h"  // esphome::binary_sensor::BinarySensor
#include "esphome/components/text_sensor/text_sensor.h"      // esphome::text_sensor::TextSensor

#include "ipc.h"
#include "opnpoolstate.h"

namespace esphome {
namespace opnpool {

    // forward declarations
struct poolstate_t;
class OpnPool;
class OpnPoolState; 

    // climate entity
class OpnPoolClimate : public climate::Climate {

  public:
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
    void set_parent(OpnPool *parent) { this->parent_ = parent; }

  protected:
    OpnPool *parent_{nullptr};
};

    // switch entity
class OpnPoolSwitch : public switch_::Switch {
  public:
    void set_circuit_id(uint8_t circuit_id) { this->circuit_id_ = circuit_id; }
    uint8_t get_circuit_id() const { return this->circuit_id_; }
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
  
  protected:
    void write_state(bool state) override;
    OpnPool *parent_{nullptr};
    uint8_t circuit_id_{0};
};

    // sensor entities
class OpnPoolSensor : public sensor::Sensor {
  public:
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    // The base class sensor::Sensor already provides publish_state(float state)
  protected:    
    OpnPool *parent_{nullptr};
};

class OpnPoolBinarySensor : public binary_sensor::BinarySensor {
  public:
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    // The base class sensor::Sensor already provides publish_state(float state)
   protected:
    OpnPool *parent_{nullptr};
};

class OpnPoolTextSensor : public text_sensor::TextSensor {
  public:
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    // The base class sensor::Sensor already provides publish_state(float state)
   protected:
    OpnPool *parent_{nullptr};
};

    // main component

class OpnPool : public Component, public uart::UARTDevice {

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

    // switch setters
    void set_pool_switch(OpnPoolSwitch *s) { pool_sw_ = s; if (s) s->set_parent(this);}
    void set_spa_switch(OpnPoolSwitch *s) { spa_sw_ = s; if (s) s->set_parent(this);}
    void set_aux1_switch(OpnPoolSwitch *s) { aux1_sw_ = s; if (s) s->set_parent(this);}
    void set_aux2_switch(OpnPoolSwitch *s) { aux2_sw_ = s; if (s) s->set_parent(this);}
    void set_feature1_switch(OpnPoolSwitch *s) { feature1_sw_ = s; if (s) s->set_parent(this);}
    void set_feature2_switch(OpnPoolSwitch *s) { feature2_sw_ = s; if (s) s->set_parent(this);}
    void set_feature3_switch(OpnPoolSwitch *s) { feature3_sw_ = s; if (s) s->set_parent(this);}
    void set_feature4_switch(OpnPoolSwitch *s) { feature4_sw_ = s; if (s) s->set_parent(this);}

    // analog sensor setters
    void set_air_temperature_sensor(OpnPoolSensor *s) { air_temp_s_ = s; if (s) s->set_parent(this); }
    void set_water_temperature_sensor(OpnPoolSensor *s) { water_temp_s_ = s; if (s) s->set_parent(this); }
    void set_pump_power_sensor(OpnPoolSensor *s) { pump_power_s_ = s; if (s) s->set_parent(this); }
    void set_pump_flow_sensor(OpnPoolSensor *s) { pump_flow_s_ = s; if (s) s->set_parent(this); }
    void set_pump_speed_sensor(OpnPoolSensor *s) { pump_speed_s_ = s; if (s) s->set_parent(this); }
    void set_chlorinator_level_sensor(OpnPoolSensor *s) { chlor_level_s_ = s; if (s) s->set_parent(this); }
    void set_chlorinator_salt_sensor(OpnPoolSensor *s) { chlor_salt_s_ = s; if (s) s->set_parent(this); }
    void set_pump_status_sensor(OpnPoolSensor *s) { pump_status_s_ = s; if (s) s->set_parent(this); }
    void set_pump_state_sensor(OpnPoolSensor *s) { pump_state_s_ = s; if (s) s->set_parent(this); }
    void set_pump_error_sensor(OpnPoolSensor *s) { pump_error_s_ = s; if (s) s->set_parent(this); }
    
    // text sensor setters
    void set_pool_sched_text_sensor(OpnPoolTextSensor *s) { pool_sched_ts_ = s; if (s) s->set_parent(this); }
    void set_spa_sched_text_sensor(OpnPoolTextSensor *s) { spa_sched_ts_ = s; if (s) s->set_parent(this); }
    void set_aux1_sched_text_sensor(OpnPoolTextSensor *s) { aux1_sched_ts_ = s; if (s) s->set_parent(this); }
    void set_aux2_sched_text_sensor(OpnPoolTextSensor *s) { aux2_sched_ts_ = s; if (s) s->set_parent(this); }
    void set_system_time_text_sensor(OpnPoolTextSensor *s) { system_time_ts_ = s; if (s) s->set_parent(this); }
    void set_controller_firmware_version_text_sensor(OpnPoolTextSensor *s) { controller_fw_ts_ = s; if (s) s->set_parent(this); }
    void set_interface_firmware_version_text_sensor(OpnPoolTextSensor *s) { interface_fw_ts_ = s; if (s) s->set_parent(this); }
    void set_pump_mode_text_sensor(OpnPoolTextSensor *s) { pump_mode_ts_ = s; if (s) s->set_parent(this); }
    void set_chlorinator_name_text_sensor(OpnPoolTextSensor *s) { chlor_name_ts_ = s; if (s) s->set_parent(this); }
    void set_chlorinator_status_text_sensor(OpnPoolTextSensor *s) { chlor_status_ts_ = s; if (s) s->set_parent(this); }

    // binary sensor setters
    void set_pump_running_binary_sensor(OpnPoolBinarySensor *s) { pump_running_bs_ = s; if (s) s->set_parent(this); }
    void set_mode_service_binary_sensor(OpnPoolBinarySensor *s) { mode_service_bs_ = s; if (s) s->set_parent(this); }
    void set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor *s) { mode_temp_inc_bs_ = s; if (s) s->set_parent(this); }
    void set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor *s) { mode_freeze_bs_ = s; if (s) s->set_parent(this); }
    void set_mode_timeout_binary_sensor(OpnPoolBinarySensor *s) { mode_timeout_bs_ = s; if (s) s->set_parent(this); }

    void write_packet(uint8_t command, const std::vector<uint8_t> &payload);
    
        // switch management
    void on_switch_command(uint8_t circuit, bool state);    
    void add_pending_switch(OpnPoolSwitch *sw, bool target_state);
    void check_pending_switches(const poolstate_t *new_state);
    
        // climate management
    void add_pending_climate(OpnPoolClimate *climate, bool has_setpoint, float setpoint_celsius, bool has_heat_src, uint8_t heat_src);
    void check_pending_climates(const poolstate_t *new_state);

        // update sensors
    void update_text_sensors(const poolstate_t *new_state);
    void update_analog_sensors(const poolstate_t *new_state);
    void update_binary_sensors(const poolstate_t *new_state);

        // climate management
    void update_climates(const poolstate_t *new_state);
    
        // helper to get climate index
    uint8_t get_climate_index(OpnPoolClimate *climate) {
        if (climate == this->pool_heater_) {
            return static_cast<uint8_t>(poolstate_thermo_typ_t::POOL);
        } else if (climate == this->spa_heater_) {
            return static_cast<uint8_t>(poolstate_thermo_typ_t::SPA);
        }
        return 0;
    }

    OpnPoolState * get_opnpool_state() { return opnPoolState_; }
    ipc_t * get_ipc() { return &ipc_; }

  protected:

    OpnPoolState * opnPoolState_{nullptr};

    ipc_t ipc_{};  // interprocess communication structure and RS485-pins

    void service_requests_from_pool(ipc_t const * const ipc);

    void parse_packet_(const std::vector<uint8_t> &data);
    std::vector<uint8_t> rx_buffer_;

    // member pointers
    OpnPoolClimate *pool_heater_{nullptr}, *spa_heater_{nullptr};
    OpnPoolSwitch *pool_sw_{nullptr}, *spa_sw_{nullptr}, *aux1_sw_{nullptr}, *aux2_sw_{nullptr};
    OpnPoolSwitch *feature1_sw_{nullptr}, *feature2_sw_{nullptr}, *feature3_sw_{nullptr}, *feature4_sw_{nullptr};
    OpnPoolSensor *air_temp_s_{nullptr}, *water_temp_s_{nullptr}, *pump_power_s_{nullptr}, *pump_flow_s_{nullptr};
    OpnPoolSensor *pump_speed_s_{nullptr}, *chlor_level_s_{nullptr}, *chlor_salt_s_{nullptr};
    OpnPoolSensor *pump_status_s_{nullptr}, *pump_state_s_{nullptr}, *pump_error_s_{nullptr};
    OpnPoolBinarySensor *pump_running_bs_{nullptr}, *mode_service_bs_{nullptr}, *mode_temp_inc_bs_{nullptr};
    OpnPoolBinarySensor *mode_freeze_bs_{nullptr}, *mode_timeout_bs_{nullptr};

    OpnPoolTextSensor *pool_sched_ts_{nullptr}, *spa_sched_ts_{nullptr}, *aux1_sched_ts_{nullptr}, *aux2_sched_ts_{nullptr};
    OpnPoolTextSensor *system_time_ts_{nullptr}, *controller_fw_ts_{nullptr}, *interface_fw_ts_{nullptr};
    OpnPoolTextSensor *pump_mode_ts_{nullptr}, *chlor_name_ts_{nullptr}, *chlor_status_ts_{nullptr};

        // track pending switch commands awaiting confirmation
    struct pending_switch_t {
        OpnPoolSwitch *  sw;
        bool             target_state;
        uint32_t         timestamp;  // for timeout handling
    };
    std::vector<pending_switch_t> pending_switches_;
    
    struct pending_climate_t {
        OpnPoolClimate *  climate;
        float             target_setpoint_celsius;
        uint8_t           target_heat_src;
        uint32_t          timestamp;
        bool              has_setpoint_change;
        bool              has_heat_src_change;
    };
    std::vector<pending_climate_t> pending_climates_;
};

} // namespace opnpool
} // namespace esphome