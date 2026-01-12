import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, switch, sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID, CONF_UNIT_OF_MEASUREMENT, CONF_DEVICE_CLASS,
    UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["uart", "climate", "switch", "sensor", "binary_sensor", "text_sensor"]
AUTO_LOAD = ["uart", "climate", "switch", "sensor", "binary_sensor", "text_sensor"]

# namespace and class definitions
opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component, uart.UARTDevice)
OpnPoolClimate = opnpool_ns.class_("OpnPoolClimate", climate.Climate)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch)
OpnPoolSensor = opnpool_ns.class_("OpnPoolSensor", sensor.Sensor)
OpnPoolBinarySensor = opnpool_ns.class_("OpnPoolBinarySensor", binary_sensor.BinarySensor)
OpnPoolTextSensor = opnpool_ns.class_("OpnPoolTextSensor", text_sensor.TextSensor)

CONF_RS485 = "rs485"
CONF_RS485_RX_PIN = "rx_pin"
CONF_RS485_TX_PIN = "tx_pin"
CONF_RS485_FLOW_CONTROL_PIN = "flow_control_pin"

# entity definitions
CONF_CLIMATES = [
    "pool_heater",
    "spa_heater"
]
CONF_SWITCHES = [  # MUST MATCH network_pool_circuit_t
    "spa", 
    "aux1",
    "aux2",
    "aux3",
    "feature1",
    "pool",
    "feature2",
    "feature3",
    "feature4"
]
CONF_ANALOG_SENSORS = [
    "air_temperature",
    "water_temperature",
    "pump_power",
    "pump_flow",
    "pump_speed",
    "chlorinator_level",
    "chlorinator_salt",
    "pump_status",
    "pump_state",
    "pump_error"
]
CONF_BINARY_SENSORS = [
    "pump_running",
    "mode_service",
    "mode_temperature_inc", 
    "mode_freeze_protection",
    "mode_timeout"
]
CONF_TEXT_SENSORS = [
    "pool_sched",
    "spa_sched",
    "aux1_sched",
    "aux2_sched", 
    "pump_mode",
    "chlorinator_name",
    "chlorinator_status",
    "system_time",
    "controller_firmware_version",
    "interface_firmware_version"
]

# Add default values and allowed parity enums where we build the CONFIG_SCHEMA.
# Example addition (insert into the CONFIG_SCHEMA mapping):
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpnPool),
    # grouped RS485 settings (preferred)
    cv.Optional(CONF_RS485): cv.Schema({
        cv.Optional(CONF_RS485_RX_PIN, default=25): cv.int_,
        cv.Optional(CONF_RS485_TX_PIN, default=26): cv.int_,
        cv.Optional(CONF_RS485_FLOW_CONTROL_PIN, default=27): cv.int_,
     }),
    **{cv.Optional(key): climate.climate_schema(OpnPoolClimate) for key in CONF_CLIMATES},
    **{cv.Optional(key): switch.switch_schema(OpnPoolSwitch) for key in CONF_SWITCHES},
    **{
        cv.Optional(key): sensor.sensor_schema(
            OpnPoolSensor,
            state_class=STATE_CLASS_MEASUREMENT,
        ) for key in CONF_ANALOG_SENSORS
    },
    **{cv.Optional(key): binary_sensor.binary_sensor_schema(OpnPoolBinarySensor) for key in CONF_BINARY_SENSORS},
    **{cv.Optional(key): text_sensor.text_sensor_schema(OpnPoolTextSensor) for key in CONF_TEXT_SENSORS},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    # instantiate the main OpnPool component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # add build flags
    cg.add_build_flag("-fmax-errors=5")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MIN=0")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MAX=256")

    # pass the RS485-setting to the OpnPool instance
    if CONF_RS485 in config:
        rs485_config = config[CONF_RS485]
        if CONF_RS485_RX_PIN in rs485_config:
            cg.add(var.set_rs485_rx_pin(rs485_config[CONF_RS485_RX_PIN]))
        if CONF_RS485_TX_PIN in rs485_config:
            cg.add(var.set_rs485_tx_pin(rs485_config[CONF_RS485_TX_PIN]))
        if CONF_RS485_FLOW_CONTROL_PIN in rs485_config:
            cg.add(var.set_rs485_flow_control_pin(rs485_config[CONF_RS485_FLOW_CONTROL_PIN]))

    # register climate entities
    for climate_key in CONF_CLIMATES:
        if climate_key in config:
            climate_entity = await climate.new_climate(config[climate_key])
            cg.add(getattr(var, f"set_{climate_key}")(climate_entity))

    # register switches
    for switch_key in CONF_SWITCHES:
        if switch_key in config:
            switch_entity = await switch.new_switch(config[switch_key])
            cg.add(getattr(var, f"set_{switch_key}_switch")(switch_entity))

    # Register analog sensors
    for sensor_key in CONF_ANALOG_SENSORS:
        if sensor_key in config:
            sensor_entity = await sensor.new_sensor(config[sensor_key])
            cg.add(getattr(var, f"set_{sensor_key}_sensor")(sensor_entity))
    
    # register binary sensors
    for binary_sensor_key in CONF_BINARY_SENSORS:
        if binary_sensor_key in config:
            bs_entity = await binary_sensor.new_binary_sensor(config[binary_sensor_key])
            cg.add(getattr(var, f"set_{binary_sensor_key}_binary_sensor")(bs_entity))

    # register text sensors
    for text_sensor_key in CONF_TEXT_SENSORS:
        if text_sensor_key in config:
            ts_entity = await text_sensor.new_text_sensor(config[text_sensor_key])
            cg.add(getattr(var, f"set_{text_sensor_key}_text_sensor")(ts_entity))
