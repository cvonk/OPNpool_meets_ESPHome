import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch, sensor, binary_sensor, text_sensor, climate
from esphome.const import (
    CONF_ID, CONF_UART_ID, CONF_UNIT_OF_MEASUREMENT, CONF_DEVICE_CLASS, UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT
)

DEPENDENCIES = ["switch", "sensor", "binary_sensor", "text_sensor", "climate"]
AUTO_LOAD = ["switch", "sensor", "binary_sensor", "text_sensor", "climate"]

# namespace and class definitions
opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch)
OpnPoolClimate = opnpool_ns.class_("OpnPoolClimate", climate.Climate)

# entity definitions
CONF_HEATERS = ["pool_heater", "spa_heater"]
CONF_SWITCHES = ["pool", "spa", "aux1", "aux2", "feature1", "feature2", "feature3", "feature4"]
CONF_TEXT_SENSORS = [
    "pool_sched", "spa_sched", "aux1_sched", "aux2_sched", 
    "system_time", "controller_firmware_version", "interface_firmware_version",
    "pump_mode", "chlorinator_name"
]
CONF_ANALOG_SENSORS = [
    "air_temperature",
    "water_temperature",
    "pump_power",
    "pump_flow",
    "pump_speed",
    "chlorinator_percentage",
    "chlorinator_salt",
    "pump_status",
    "pump_state",
    "pump_error",
    "chlorinator_status"
]
CONF_BINARY_SENSORS = [
    "pump_running", "mode_service", "mode_temperature_inc", 
    "mode_freeze_protection", "mode_timeout"
]
# granular control of debug levels
CONF_DEBUG_MODULES = [
    "datalink", "network", "pool_state",
    "pool_task", "mqtt_task", "hass_task"
]
DEBUG_LEVELS = {
    "NONE": opnpool_ns.DEBUG_NONE,
    "ERROR": opnpool_ns.DEBUG_ERROR,
    "WARN": opnpool_ns.DEBUG_WARN,
    "INFO": opnpool_ns.DEBUG_INFO,
    "DEBUG": opnpool_ns.DEBUG_DEBUG,
    "VERBOSE": opnpool_ns.DEBUG_VERBOSE,
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpnPool),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    **{cv.Optional(key): climate.climate_schema(OpnPoolClimate) for key in CONF_HEATERS},
    **{cv.Optional(key): switch.switch_schema(OpnPoolSwitch) for key in CONF_SWITCHES},
    **{cv.Optional(key): text_sensor.text_sensor_schema() for key in CONF_TEXT_SENSORS},
    **{cv.Optional(key): sensor.sensor_schema() for key in CONF_ANALOG_SENSORS},
    **{cv.Optional(key): binary_sensor.binary_sensor_schema() for key in CONF_BINARY_SENSORS},
    **{cv.Optional(f"{key}_debug", default="INFO"): cv.enum(DEBUG_LEVELS, upper=True) 
       for key in CONF_DEBUG_MODULES},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    
    # instantiate the main OpnPool component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # link OpnPool component to the UART-bus
    await uart.register_uart_device(var, config)

    for heater in CONF_HEATERS:
        if heater in config:
            clm = cg.new_Pvariable(config[heater][CONF_ID])
            await climate.register_climate(clm, config[heater])
            cg.add(getattr(var, f"set_{heater}")(clm))

    for key in CONF_SWITCHES:
        if key in config:
            sw = cg.new_Pvariable(config[key][CONF_ID])
            await switch.register_switch(sw, config[key])
            cg.add(getattr(var, f"set_{key}_switch")(sw))

    for key in CONF_ANALOG_SENSORS:
        if key in config:
            # create the sensor object with YAML-settings (device_class, units, etc.)
            sens = await sensor.new_sensor(config[key])
            # call the corresponding C++ setter: e.g., var->set_water_temperature_sensor(sens)
            cg.add(getattr(var, f"set_{key}_sensor")(sens))
    for key in CONF_TEXT_SENSORS:
        if key in config:
            tsens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_text_sensor")(tsens))

    for key in CONF_BINARY_SENSORS:
        if key in config:
            bsens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_binary_sensor")(bsens))

    for m in CONF_DEBUG_MODULES:
        conf_key = f"{m}_debug"
        if conf_key in config:
            cg.add(getattr(var, f"set_{m}_debug")(config[conf_key]))            