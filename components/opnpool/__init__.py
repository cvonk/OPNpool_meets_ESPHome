# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Coert Vonk

"""
@file __init__.py
@author Coert Vonk (@cvonk on GitHub)
@brief OPNpool - ESPHome Python codegen for the OPNpool component.
 
@copyright Copyright (c) 2026, Coert Vonk

@details
This file defines the ESPHome code generation logic for the OPNpool component, which integrates
an OPNpool interface with the ESPHome ecosystem. It declares the configuration schema, entity
types, and code generation routines for climate, switch, sensor, binary sensor, and text sensor
entities associated with the pool controller.

Responsibilities include:
- Defining the configuration schema for all supported pool entities and RS485 hardware settings.
- Registering all subcomponents and their C++ counterparts for code generation.
- Dynamically extracting the firmware version from Git or ESPHome version for build metadata.
- Adding required build flags and source files for the C++ implementation.
- Providing async routines to instantiate and link all entities to the main OpnPool component.

WARNING: The script directly edits the `opnpool.h` header file to keep the enums in opnpool.h 
consistent with CONF_* in this file.

This module enables seamless integration of pool automation hardware into ESPHome YAML
configurations, supporting flexible entity mapping and robust build-time configuration.
"""

import sys
if sys.version_info < (3, 6):
    raise RuntimeError("OPNpool ESPHome component requires Python 3.6 or newer.")

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

CONF_CLIMATES = [  # used to overwrite ClimateId enum in opnpool.h
    "pool_climate",
    "spa_climate"
]
CONF_SWITCHES = [  # used to overwrite SwitchId enum in opnpool.h
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
CONF_ANALOG_SENSORS = [  # used to overwrite SensorId enum in opnpool.h
    "air_temperature",
    "water_temperature",
    "pump_power",
    "pump_flow",
    "pump_speed",
    "chlorinator_level",
    "chlorinator_salt",
    "pump_error"
]
CONF_BINARY_SENSORS = [  # used to overwrite BinarySensorId enum in opnpool.h
    "pump_running",
    "mode_service",
    "mode_temperature_inc", 
    "mode_freeze_protection",
    "mode_timeout"
]
CONF_TEXT_SENSORS = [  # used to overwrite TextSensorId enum in opnpool.h
    "pool_sched",
    "spa_sched",
    "pump_mode",
    "pump_state",
    "chlorinator_name",
    "chlorinator_status",
    "system_time",
    "controller_firmware",
    "interface_firmware"
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
            import logging
            logging.warning("OPNpool: Could not determine firmware version from Git or ESPHome. Using 'unknown'.")
            version = "unknown"

    cg.add_build_flag(f"-DGIT_HASH=\\\"{version}\\\"")
    
    # RS485 configuration
    rs485_config = config[CONF_RS485]
    cg.add(var.set_rs485_pins(rs485_config[CONF_RS485_RX_PIN], rs485_config[CONF_RS485_TX_PIN], rs485_config[CONF_RS485_FLOW_CONTROL_PIN]))

    # register climate entities (constructor injection)
    for idx, climate_key in enumerate(CONF_CLIMATES):
        entity_cfg = config[climate_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        climate_entity = cg.new_Pvariable(entity_cfg[CONF_ID], var, idx)
        await climate.register_climate(climate_entity, entity_cfg)
        cg.add(getattr(var, f"set_{climate_key}")(climate_entity))

    # register switches (constructor injection)
    for idx, switch_key in enumerate(CONF_SWITCHES):
        entity_cfg = config[switch_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        switch_entity = cg.new_Pvariable(entity_cfg[CONF_ID], var, idx)
        await switch.register_switch(switch_entity, entity_cfg)
        cg.add(getattr(var, f"set_{switch_key}_switch")(switch_entity))

    # register analog sensors (constructor injection)
    for idx, sensor_key in enumerate(CONF_ANALOG_SENSORS):
        entity_cfg = config[sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        sensor_entity = cg.new_Pvariable(entity_cfg[CONF_ID], var, idx)
        await sensor.register_sensor(sensor_entity, entity_cfg)
        cg.add(getattr(var, f"set_{sensor_key}_sensor")(sensor_entity))

    # register binary sensors (constructor injection)
    for idx, binary_sensor_key in enumerate(CONF_BINARY_SENSORS):
        entity_cfg = config[binary_sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        bs_entity = cg.new_Pvariable(entity_cfg[CONF_ID], var, idx)
        await binary_sensor.register_binary_sensor(bs_entity, entity_cfg)
        cg.add(getattr(var, f"set_{binary_sensor_key}_binary_sensor")(bs_entity))

    # register text sensors (constructor injection)
    for idx, text_sensor_key in enumerate(CONF_TEXT_SENSORS):
        entity_cfg = config[text_sensor_key]
        if CONF_ID not in entity_cfg:
            entity_cfg[CONF_ID] = cg.new_id()
        ts_entity = cg.new_Pvariable(entity_cfg[CONF_ID], var, idx)
        await text_sensor.register_text_sensor(ts_entity, entity_cfg)
        cg.add(getattr(var, f"set_{text_sensor_key}_text_sensor")(ts_entity))

# replace the enums in opnpool.h to keep them consistent with CONF_* in this file

import os
import re

ENTITY_ENUMS = {
    "ClimateId": CONF_CLIMATES,
    "SwitchId": CONF_SWITCHES,
    "SensorId": CONF_ANALOG_SENSORS,
    "BinarySensorId": CONF_BINARY_SENSORS,
    "TextSensorId": CONF_TEXT_SENSORS,
}

def generate_enum(enum_name, items):
    lines = [f"enum class {enum_name} : uint8_t {{"]
    for idx, name in enumerate(items):
        lines.append(f"    {name.upper()} = {idx},")
    lines.append("    COUNT")
    lines.append("};")
    return "\n".join(lines)

def update_header(header_path, entity_enums):
    import tempfile
    import shutil

    with open(header_path, "r") as f:
        content = f.read()
    for enum_name, items in entity_enums.items():
        pattern = rf"enum class {enum_name} : uint8_t \{{.*?\}};"
        new_enum = generate_enum(enum_name, items)
        content = re.sub(pattern, new_enum, content, flags=re.DOTALL)

    # write to a temporary file first
    dir_name = os.path.dirname(header_path)
    with tempfile.NamedTemporaryFile("w", dir=dir_name, delete=False) as tmp_file:
        tmp_file.write(content)
        temp_path = tmp_file.name

    # atomically replace the original file
    shutil.move(temp_path, header_path)

if __name__ == "__main__":
    # use a relative path from this script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    header_path = os.path.join(script_dir, "opnpool.h")
    update_header(header_path, ENTITY_ENUMS)
