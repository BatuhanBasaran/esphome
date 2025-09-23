import esphome.config_validation as cv

from ..bme69x_base import (
    CONFIG_SCHEMA as CONFIG_SCHEMA_BASE,
    BME69x_Component,
    to_code as to_code_base,
)

AUTO_LOAD = ["bme69x_base"]
DEPENDENCIES = ["i2c"]


CONFIG_SCHEMA = CONFIG_SCHEMA_BASE.extend(
    {
        cv.GenerateID(): cv.declare_id(BME69x_Component),
    }
)


async def to_code(config):
    await to_code_base(config)
