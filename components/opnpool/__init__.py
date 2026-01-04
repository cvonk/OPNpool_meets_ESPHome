import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, switch, sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID, CONF_UART_ID, CONF_UNIT_OF_MEASUREMENT, CONF_DEVICE_CLASS,
    UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["uart", "climate", "switch", "sensor", "binary_sensor", "text_sensor"]
AUTO_LOAD = ["uart", "climate", "switch", "sensor", "binary_sensor", "text_sensor"]

# namespace and class definitions
opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component)
OpnPoolClimate = opnpool_ns.class_("OpnPoolClimate", climate.Climate)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch)
OpnPoolSensor = opnpool_ns.class_("OpnPoolSensor", sensor.Sensor)
OpnPoolBinarySensor = opnpool_ns.class_("OpnPoolBinarySensor", binary_sensor.BinarySensor)
OpnPoolTextSensor = opnpool_ns.class_("OpnPoolTextSensor", text_sensor.TextSensor)

# entity definitions
CONF_CLIMATES = [
    "pool_heater",
    "spa_heater"
]
CONF_SWITCHES = [
    "pool",
    "spa",
    "aux1",
    "aux2",
    "feature1",
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
# granular control of debug levels
CONF_DEBUG_MODULES = [
    "datalink",
    "network",
    "pool_state",
    "pool_task",
    "mqtt_task",
    "hass_task"
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
    #**{
    #    cv.Optional(key): number.number_schema(
    #        OpnPoolNumber,
    #    ).extend({
    #        # setting defaults here prevents the YAML from getting cluttered
    #        cv.Optional(CONF_MIN_VALUE, default=0.0): cv.float_,
    #        cv.Optional(CONF_MAX_VALUE, default=100.0): cv.float_,
    #        cv.Optional(CONF_STEP, default=1.0): cv.float_,
    #    }) for key in CONF_NUMBERS
    #},
    **{cv.Optional(f"{key}_debug", default="INFO"): cv.enum(DEBUG_LEVELS, upper=True) for key in CONF_DEBUG_MODULES},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    
    # instantiate the main OpnPool component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # link OpnPool component to the UART-bus
    await uart.register_uart_device(var, config)

    for heater in CONF_CLIMATES:
        if heater in config:
            obj = cg.new_Pvariable(config[heater][CONF_ID])
            await climate.register_climate(obj, config[heater])
            cg.add(getattr(var, f"set_{heater}")(obj))

    for key in CONF_SWITCHES:
        if key in config:
            obj = cg.new_Pvariable(config[key][CONF_ID])
            await switch.register_switch(obj, config[key])
            cg.add(getattr(var, f"set_{key}_switch")(obj))

    #for key in CONF_ANALOG_SENSORS:
    #    if key in config:
    #        obj = await sensor.new_sensor(config[key])
    #        await sensor.register_sensor(obj, config[key])
    #        cg.add(getattr(var, f"set_{key}_sensor")(obj))

    for key in CONF_ANALOG_SENSORS:
        if key in config:
            conf = config[key]
            # if for some reason the schema didn't inject this,
            # we ensure it's there before calling new_sensor.
            if "force_update" not in conf:
                conf["force_update"] = False
            obj = await sensor.new_sensor(conf)
            cg.add(getattr(var, f"set_{key}_sensor")(obj))

    for key in CONF_BINARY_SENSORS:
        if key in config:
            obj = await binary_sensor.new_binary_sensor(config[key])
            await binary_sensor.register_binary_sensor(obj, config[key])
            cg.add(getattr(var, f"set_{key}_binary_sensor")(obj))

    for key in CONF_TEXT_SENSORS:
        if key in config:
            obj = await text_sensor.new_text_sensor(config[key])
            await text_sensor.register_text_sensor(obj, config[key])
            cg.add(getattr(var, f"set_{key}_text_sensor")(obj))

    for key in CONF_DEBUG_MODULES:
        conf_key = f"{key}_debug"
        if conf_key in config:
            cg.add(getattr(var, f"set_{key}_debug")(config[conf_key]))            