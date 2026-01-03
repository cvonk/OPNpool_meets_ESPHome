import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch, sensor, binary_sensor, text_sensor, climate
from esphome.const import (
    CONF_ID, UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT
)

DEPENDENCIES = ["switch", "sensor", "binary_sensor", "text_sensor", "climate"]
AUTO_LOAD = ["switch", "sensor", "binary_sensor", "text_sensor", "climate"]

opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch)
OpnPoolClimate = opnpool_ns.class_("OpnPoolClimate", climate.Climate)

# Entity Definitions
CONF_SWITCHES = ["pool", "spa", "aux1", "aux2", "flt1", "flt2", "flt3", "flt4"]
CONF_TEXT_SENSORS = [
    "pool_sched", "spa_sched", "aux1_sched", "aux2_sched", 
    "system_time", "controller_firmware_version", "interface_firmware_version",
    "pump_mode", "chlorinator_name"
]
CONF_ANALOG_SENSORS = {
    "air_temperature": "°C", "water_temperature": "°C",
    "pump_power": UNIT_WATT, "pump_flow": "gpm", "pump_speed": "rpm",
    "chlorinator_percentage": UNIT_PERCENT, "chlorinator_salt": "ppm",
    "pump_status": "", "pump_state": "", "pump_error": "", "chlorinator_status": ""
}
CONF_BINARY_SENSORS = [
    "pump_running", "mode_service", "mode_temperature_inc", 
    "mode_freeze_protection", "mode_timeout"
]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpnPool),
    cv.Optional("pool_heater"): climate.climate_schema(OpnPoolClimate),
    cv.Optional("spa_heater"): climate.climate_schema(OpnPoolClimate),
    **{cv.Optional(s): switch.switch_schema(OpnPoolSwitch) for s in CONF_SWITCHES},
    **{cv.Optional(ts): text_sensor.text_sensor_schema() for ts in CONF_TEXT_SENSORS},
    **{cv.Optional(as_): sensor.sensor_schema(unit_of_measurement=u, state_class=STATE_CLASS_MEASUREMENT) 
       for as_, u in CONF_ANALOG_SENSORS.items()},
    **{cv.Optional(bs): binary_sensor.binary_sensor_schema() for bs in CONF_BINARY_SENSORS},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Climate Registration
    for heater in ["pool_heater", "spa_heater"]:
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
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    for key in CONF_TEXT_SENSORS:
        if key in config:
            tsens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_text_sensor")(tsens))

    for key in CONF_BINARY_SENSORS:
        if key in config:
            bsens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_binary_sensor")(bsens))