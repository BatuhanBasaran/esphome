#include <cstddef>
#include <cstdint>

#include "bme69x_i2c.h"
#include "esphome/components/i2c/i2c.h"
#include "../bme69x_base/bme69x_base.h"

namespace esphome {
namespace bme69x_i2c {

bool BME69x_I2C_Component::read_byte(uint8_t a_register, uint8_t *data) {
  return I2CDevice::read_byte(a_register, data);
};
bool BME69x_I2C_Component::write_byte(uint8_t a_register, uint8_t data) {
  return I2CDevice::write_byte(a_register, data);
};
bool BME69x_I2C_Component::write_bytes(uint8_t a_register, const uint8_t *data, uint8_t len) {
  return I2CDevice::write_bytes(a_register, data, len);
};
bool BME69x_I2C_Component::read_bytes(uint8_t a_register, uint8_t *data, size_t len) {
  return I2CDevice::read_bytes(a_register, data, len);
};
bool BME69x_I2C_Component::read_byte_16(uint8_t a_register, uint16_t *data) {
  return I2CDevice::read_byte_16(a_register, data);
};

void BME69x_I2C_Component::dump_config() {
  LOG_I2C_DEVICE(this);
  BME69x_Component::dump_config();
}

}  // namespace bme69x_i2c
}  // namespace esphome
