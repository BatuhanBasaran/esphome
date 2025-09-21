#pragma once

#include "esphome/components/bme69x_base/bme69x_base.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bme69x_i2c {

static const char *const TAG = "bme69x_i2c.sensor";

class BME69x_I2C_Component : public esphome::bme69x_base::BME69x_Component, public i2c::I2CDevice {
  bool read_byte(uint8_t a_register, uint8_t *data) override;
  bool write_byte(uint8_t a_register, uint8_t data) override;
  bool write_bytes(uint8_t a_register, const uint8_t *data, uint8_t len) override;
  bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) override;
  bool read_byte_16(uint8_t a_register, uint16_t *data) override;
  void dump_config() override;
};

}  // namespace bme69x_i2c
}  // namespace esphome
