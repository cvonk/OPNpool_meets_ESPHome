import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch, binary_sensor
from esphome import pins
from esphome.const import CONF_ID, CONF_PIN

# --- FIX: Tell ESPHome to include these components in the build ---
DEPENDENCIES = ["switch", "binary_sensor"]
AUTO_LOAD = ["switch", "binary_sensor"]
# ------------------------------------------------------------------

opnpool_ns = cg.esphome_ns.namespace("opnpool")
OpnPool = opnpool_ns.class_("OpnPool", cg.Component)
OpnPoolSwitch = opnpool_ns.class_("OpnPoolSwitch", switch.Switch)

CONF_SWITCHES = "switches"
CONF_SENSORS = "sensors"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpnPool),
    cv.Required(CONF_SWITCHES): cv.All(
        cv.ensure_list(switch.switch_schema(OpnPoolSwitch).extend({
            cv.Required(CONF_PIN): pins.gpio_output_pin_schema,
        })),
        cv.Length(min=4, max=4)
    ),
    cv.Required(CONF_SENSORS): cv.All(
        cv.ensure_list(binary_sensor.binary_sensor_schema().extend({
            cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
        })),
        cv.Length(min=3, max=3)
    ),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for sw_conf in config[CONF_SWITCHES]:
        sw = cg.new_Pvariable(sw_conf[CONF_ID])
        await switch.register_switch(sw, sw_conf)
        pin = await cg.gpio_pin_expression(sw_conf[CONF_PIN])
        cg.add(var.add_switch(sw, pin))

    for sens_conf in config[CONF_SENSORS]:
        # Use new_binary_sensor to properly initialize the sensor
        sens = await binary_sensor.new_binary_sensor(sens_conf)
        pin = await cg.gpio_pin_expression(sens_conf[CONF_PIN])
        cg.add(var.add_binary_sensor(sens, pin))