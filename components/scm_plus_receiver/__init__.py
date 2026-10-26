import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_BRIGHTNESS, CONF_TOPIC
from esphome.components import remote_base, light

scm_plus_ns = cg.esphome_ns.namespace("scm_plus")
ScmPlusDecoder = scm_plus_ns.class_("ScmPlusDecoder", cg.Component)

CONF_RECEIVER_ID = "receiver_id"
CONF_LED_ID = "led_id"
CONF_METER_ID = "meter_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ScmPlusDecoder),
    cv.Required(CONF_RECEIVER_ID): cv.use_id(remote_base.RemoteReceiverBase),
    cv.Required(CONF_LED_ID): cv.use_id(light.LightState),
    cv.Optional(CONF_BRIGHTNESS, default="10%"): cv.percentage,
    cv.Optional(CONF_METER_ID): cv.ensure_list(cv.uint32_t),
    cv.Optional(CONF_TOPIC): cv.string,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
    led_light = await cg.get_variable(config[CONF_LED_ID])
    
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_BRIGHTNESS])
    await cg.register_component(var, config)
    
    cg.add(var.set_status_led(led_light))
    
    if CONF_METER_ID in config:
        for id_val in config[CONF_METER_ID]:
            cg.add(var.add_target_meter_id(id_val))
            
    if CONF_TOPIC in config:
        cg.add(var.set_custom_base_topic(config[CONF_TOPIC]))
            
    remote_base.add_listener(receiver, var)
    cg.add_global(cg.RawStatement('#include "components/scm_plus_receiver/scm_plus_decoder.h"'))
