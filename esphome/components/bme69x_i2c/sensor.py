import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv

from ..bme69x_base import CONFIG_SCHEMA_BASE, to_code_base

AUTO_LOAD = ["bme69x_base"]
DEPENDENCIES = ["i2c"]

bme69x_ns = cg.esphome_ns.namespace("bme69x_i2c")
BME69x_I2C_Component = bme69x_ns.class_(
    "BME69x_I2C_Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = CONFIG_SCHEMA_BASE.extend(
    {
        cv.GenerateID(): cv.declare_id(BME69x_I2C_Component),
        cv.Optional(i2c.CONF_ADDRESS, default=0x76): cv.one_of(0x76, 0x77, int=True),
    }
).extend(i2c.i2c_device_schema(default_address=0x76))


async def to_code(config):
    var = await to_code_base(config)
    await i2c.register_i2c_device(var, config)
