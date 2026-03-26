import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import Esp32ppc, CONF_ESP32PPC_ID, esp32ppc_ns

DEPENDENCIES = ["esp32ppc"]

CONF_CLAMPED = "clamped"

CounterClampSwitch = esp32ppc_ns.class_("CounterClampSwitch", switch.Switch)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ESP32PPC_ID): cv.use_id(Esp32ppc),
        cv.Optional(CONF_CLAMPED): switch.switch_schema(
            CounterClampSwitch,
            block_inverted=True,
            default_restore_mode="RESTORE_DEFAULT_OFF",
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:math-compass",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ESP32PPC_ID])
    if CONF_CLAMPED in config:
        conf = config[CONF_CLAMPED]
        clamped_switch = await switch.new_switch(conf, hub)
        cg.add(hub.set_clamped_switch(clamped_switch))
        cg.add(hub.set_clamped_mode(False))
