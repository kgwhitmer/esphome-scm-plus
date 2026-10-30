import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_BRIGHTNESS, CONF_TOPIC
from esphome.components import remote_base, light
from esphome import automation

DEPENDENCIES = ["json"]

scm_plus_ns = cg.esphome_ns.namespace("scm_plus")
ScmPlusDecoder = scm_plus_ns.class_("ScmPlusDecoder", cg.Component)

CONF_RECEIVER_ID = "receiver_id"
CONF_LED_ID = "led_id"
CONF_METER_ID = "meter_id"
CONF_ON_JSON = "on_json"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ScmPlusDecoder),
    cv.Required(CONF_RECEIVER_ID): cv.use_id(remote_base.RemoteReceiverBase),
    cv.Optional(CONF_LED_ID): cv.use_id(light.LightState),
    cv.Optional(CONF_BRIGHTNESS, default="10%"): cv.percentage,
    cv.Optional(CONF_METER_ID): cv.ensure_list(cv.uint32_t),
    cv.Optional(CONF_TOPIC): cv.string,
    cv.Optional(CONF_ON_JSON): automation.validate_automation({}),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
    
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_BRIGHTNESS])
    await cg.register_component(var, config)
    
    if (led_id := config.get(CONF_LED_ID)) is not None:
        led_light = await cg.get_variable(led_id)
        cg.add(var.set_status_led(led_light))
    
    if CONF_METER_ID in config:
        for id_val in config[CONF_METER_ID]:
            cg.add(var.add_target_meter_id(id_val))
            
    if CONF_TOPIC in config:
        cg.add(var.set_custom_base_topic(config[CONF_TOPIC]))
        
    for conf in config.get(CONF_ON_JSON, []):
        await automation.build_callback_automation(
            var, "add_on_json_callback", [(cg.JsonObject, "root")], conf
        )
            
    remote_base.add_listener(receiver, var)
    cg.add_global(cg.RawStatement('#include "components/scm_plus_receiver/scm_plus_decoder.h"'))
