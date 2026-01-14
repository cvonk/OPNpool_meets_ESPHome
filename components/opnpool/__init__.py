import os
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, switch, sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID, CONF_UNIT_OF_MEASUREMENT, CONF_DEVICE_CLASS,
    UNIT_WATT, UNIT_PERCENT, STATE_CLASS_MEASUREMENT,
)

# Remove 'uart' from dependencies
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
    "pump_mode",
    "pump_state",
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
    # RS485 settings (required, but with defaults)
    cv.Optional(CONF_RS485, default={}): cv.Schema({
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
}).extend(cv.COMPONENT_SCHEMA)  # Remove .extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    # instantiate the main OpnPool component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Add all source files to build
    cg.add_library("ESP32", None, "freertos")
    
    # Add component source files
    component_dir = os.path.dirname(__file__)
    
    # C++ entity implementation files
    cg.add_platformio_option("build_src_filter", [
        "+<*>",
        "+<esphome/components/opnpool/*.cpp>",
    ])
    
    # Or explicitly add each file:
    # This ensures the entity implementation files are compiled
    
    # add build flags
    cg.add_build_flag("-fmax-errors=5")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MIN=0")
    cg.add_build_flag("-DMAGIC_ENUM_RANGE_MAX=256")

    # Add interface firmware version
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
    
    # RS485 configuration (always set due to default={})
    rs485_config = config[CONF_RS485]
    cg.add(var.set_rs485_pins(rs485_config[CONF_RS485_RX_PIN], rs485_config[CONF_RS485_TX_PIN], rs485_config[CONF_RS485_FLOW_CONTROL_PIN]))

    # Register climate entities (only if present)
    for climate_key in CONF_CLIMATES:
        if climate_key in config:
            climate_entity = await climate.new_climate(config[climate_key])
            cg.add(getattr(var, f"set_{climate_key}")(climate_entity))

    # Register switches (only if present)
    for switch_key in CONF_SWITCHES:
        if switch_key in config:
            switch_entity = await switch.new_switch(config[switch_key])
            cg.add(getattr(var, f"set_{switch_key}_switch")(switch_entity))

    # Register analog sensors (only if present)
    for sensor_key in CONF_ANALOG_SENSORS:
        if sensor_key in config:
            sensor_entity = await sensor.new_sensor(config[sensor_key])
            cg.add(getattr(var, f"set_{sensor_key}_sensor")(sensor_entity))

    # Register binary sensors (only if present)
    for binary_sensor_key in CONF_BINARY_SENSORS:
        if binary_sensor_key in config:
            bs_entity = await binary_sensor.new_binary_sensor(config[binary_sensor_key])
            cg.add(getattr(var, f"set_{binary_sensor_key}_binary_sensor")(bs_entity))

    # Register text sensors (only if present)
    for text_sensor_key in CONF_TEXT_SENSORS:
        if text_sensor_key in config:
            ts_entity = await text_sensor.new_text_sensor(config[text_sensor_key])
            cg.add(getattr(var, f"set_{text_sensor_key}_text_sensor")(ts_entity))
