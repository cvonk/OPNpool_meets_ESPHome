import os
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, switch, sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID, CONF_UNIT_OF_MEASUREMENT, CONF_DEVICE_CLASS,
    UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["climate", "switch", "sensor", "binary_sensor", "text_sensor"]
AUTO_LOAD = ["climate", "switch", "sensor", "binary_sensor", "text_sensor"]

# namespace and class definitions
opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component)
OpnPoolClimate = opnpool_ns.class_("OpnPoolClimate", climate.Climate, cg.Component)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch, cg.Component)
OpnPoolSensor = opnpool_ns.class_("OpnPoolSensor", sensor.Sensor, cg.Component)
OpnPoolBinarySensor = opnpool_ns.class_("OpnPoolBinarySensor", binary_sensor.BinarySensor, cg.Component)
OpnPoolTextSensor = opnpool_ns.class_("OpnPoolTextSensor", text_sensor.TextSensor, cg.Component)

CONF_RS485 = "rs485"
CONF_RS485_RX_PIN = "rx_pin"
CONF_RS485_TX_PIN = "tx_pin"
CONF_RS485_FLOW_CONTROL_PIN = "flow_control_pin"

CONF_CLIMATES = [  # MUST MATCH ClimateId in opnpool.h
    "pool_climate",
    "spa_climate"
]
CONF_SWITCHES = [  # MUST MATCH SwitchId in opnpool.h
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
CONF_ANALOG_SENSORS = [  # MUST MATCH SensorId in opnpool.h
    "air_temperature",
    "water_temperature",
    "pump_power",
    "pump_flow",
    "pump_speed",
    "chlorinator_level",
    "chlorinator_salt",
    "pump_error"
]
CONF_BINARY_SENSORS = [  # MUST MATCH BinarySensorId in opnpool.h
    "pump_running",
    "mode_service",
    "mode_temperature_inc", 
    "mode_freeze_protection",
    "mode_timeout"
]
CONF_TEXT_SENSORS = [  # MUST MATCH TextSensorId in opnpool.h
    "pool_sched",
    "spa_sched",
    "pump_mode",
    "pump_state",
    "chlorinator_name",
    "chlorinator_status",
    "system_time",
    "controller_firmware_version",
    "interface_firmware_version"
]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpnPool),
    # RS485 settings (required, but with defaults)
    cv.Optional(CONF_RS485, default={}): cv.Schema({
        cv.Optional(CONF_RS485_RX_PIN, default=25): cv.int_,
        cv.Optional(CONF_RS485_TX_PIN, default=26): cv.int_,
        cv.Optional(CONF_RS485_FLOW_CONTROL_PIN, default=27): cv.int_,
    }),
    **{
        cv.Optional(key, default={"name": key.replace("_", " ").title()}): climate.climate_schema(OpnPoolClimate).extend({
            cv.GenerateID(): cv.declare_id(OpnPoolClimate)
        }) for key in CONF_CLIMATES
    },
    **{
        cv.Optional(key, default={"name": key.replace("_", " ").title()}): switch.switch_schema(OpnPoolSwitch).extend({
            cv.GenerateID(): cv.declare_id(OpnPoolSwitch)
        }) for key in CONF_SWITCHES
    },
    **{
        cv.Optional(key, default={"name": key.replace("_", " ").title()}): sensor.sensor_schema(OpnPoolSensor).extend({
            cv.GenerateID(): cv.declare_id(OpnPoolSensor)
        }) for key in CONF_ANALOG_SENSORS
    },
    **{
        cv.Optional(key, default={"name": key.replace("_", " ").title()}): binary_sensor.binary_sensor_schema(OpnPoolBinarySensor).extend({
            cv.GenerateID(): cv.declare_id(OpnPoolBinarySensor)
        }) for key in CONF_BINARY_SENSORS
    },
    **{
        cv.Optional(key, default={"name": key.replace("_", " ").title()}): text_sensor.text_sensor_schema(OpnPoolTextSensor).extend({
            cv.GenerateID(): cv.declare_id(OpnPoolTextSensor)
        }) for key in CONF_TEXT_SENSORS
    },
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # instantiate the main OpnPool component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Add all source files to build
    cg.add_library("ESP32", None, "freertos")
    #cg.add_library("OPNPool", None, "opnpool")
    
    # Add component source files
    component_dir = os.path.dirname(__file__)
    
    # C++ entity implementation files
    cg.add_platformio_option("build_src_filter", [
        "+<*>",
        "+<esphome/components/opnpool/*.cpp>",
    ])
    
    # add build flags
    cg.add_build_flag("-fmax-errors=5")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MIN=0")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MAX=256")

    # interface firmware version
    import subprocess    
    version = "unknown"
    try:
        component_dir = os.path.dirname(os.path.abspath(__file__))
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=component_dir,
            stderr=subprocess.DEVNULL
        ).decode('ascii').strip()
        
        try:
            subprocess.check_output(
                ['git', 'diff', '--quiet'],
                cwd=component_dir,
                stderr=subprocess.DEVNULL
            )
            dirty = ""
        except subprocess.CalledProcessError:
            dirty = "-dirty"
        
        version = f"git-{git_hash}{dirty}"
    except:
        try:
            from esphome.const import __version__ as ESPHOME_VERSION
            version = f"esphome-{ESPHOME_VERSION}"
        except:
            pass
    
    cg.add_build_flag(f"-DINTERFACE_FW_VERSION=\\\"{version}\\\"")
    
    # RS485 configuration
    rs485_config = config[CONF_RS485]
    cg.add(var.set_rs485_pins(rs485_config[CONF_RS485_RX_PIN], rs485_config[CONF_RS485_TX_PIN], rs485_config[CONF_RS485_FLOW_CONTROL_PIN]))

    # Register climate entities
    for climate_key in CONF_CLIMATES:
        entity_cfg = config[climate_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        climate_entity = await climate.new_climate(entity_cfg)
        cg.add(getattr(var, f"set_{climate_key}")(climate_entity))

    # Register switches
    for switch_key in CONF_SWITCHES:
        entity_cfg = config[switch_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        switch_entity = await switch.new_switch(entity_cfg)
        cg.add(getattr(var, f"set_{switch_key}_switch")(switch_entity))

    # Register analog sensors
    for sensor_key in CONF_ANALOG_SENSORS:
        entity_cfg = config[sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        sensor_entity = await sensor.new_sensor(entity_cfg)
        cg.add(getattr(var, f"set_{sensor_key}_sensor")(sensor_entity))

    # Register binary sensors
    for binary_sensor_key in CONF_BINARY_SENSORS:
        entity_cfg = config[binary_sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        bs_entity = await binary_sensor.new_binary_sensor(entity_cfg)
        cg.add(getattr(var, f"set_{binary_sensor_key}_binary_sensor")(bs_entity))

    # Register text sensors
    for text_sensor_key in CONF_TEXT_SENSORS:
        entity_cfg = config[text_sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        ts_entity = await text_sensor.new_text_sensor(entity_cfg)
        cg.add(getattr(var, f"set_{text_sensor_key}_text_sensor")(ts_entity))