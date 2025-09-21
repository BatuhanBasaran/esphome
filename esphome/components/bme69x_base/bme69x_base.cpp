#include "bme69x_base.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bme69x_base {

static const char *const TAG = "bme69x.sensor";

static const uint8_t BME69X_REG_CHIP_ID = 0xD0;
static const uint8_t BME69X_REG_SOFT_RESET = 0xE0;
static const uint8_t BME69X_SOFT_RESET_CMD = 0xB6;
static const uint8_t BME69X_REG_VARIANT_ID = 0xF0;
static const uint8_t BME69X_LEN_COEFF_ALL = 42;
static const uint8_t BME69X_REG_COEFF1 = 0x8A;
static const uint8_t BME69X_LEN_COEFF1 = 23;
static const uint8_t BME69X_REG_COEFF2 = 0xE1;
static const uint8_t BME69X_LEN_COEFF2 = 14;
static const uint8_t BME69X_REG_COEFF3 = 0x00;
static const uint8_t BME69X_LEN_COEFF3 = 5;
static const uint8_t BME69X_IDX_DO_C_MSB = 32;
static const uint8_t BME69X_IDX_DO_C_LSB = 31;
static const uint8_t BME69X_IDX_DTK1_C_MSB = 1;
static const uint8_t BME69X_IDX_DTK1_C_LSB = 0;
static const uint8_t BME69X_IDX_DTK2_C = 2;
static const uint8_t BME69X_IDX_S_C_MSB = 5;
static const uint8_t BME69X_IDX_S_C_LSB = 4;
static const uint8_t BME69X_IDX_TK1S_C_MSB = 7;
static const uint8_t BME69X_IDX_TK1S_C_LSB = 6;
static const uint8_t BME69X_IDX_TK2S_C = 8;
static const uint8_t BME69X_IDX_TK3S_C = 9;
static const uint8_t BME69X_IDX_O_C_MSB = 11;
static const uint8_t BME69X_IDX_O_C_LSB = 10;
static const uint8_t BME69X_IDX_TK10_C_MSB = 13;
static const uint8_t BME69X_IDX_TK10_C_LSB = 12;
static const uint8_t BME69X_IDX_TK20_C = 14;
static const uint8_t BME69X_IDX_TK30_C = 15;
static const uint8_t BME69X_IDX_NLS_C_MSB = 19;
static const uint8_t BME69X_IDX_NLS_C_LSB = 18;
static const uint8_t BME69X_IDX_TKNLS_C = 20;
static const uint8_t BME69X_IDX_NLS3_C = 21;
static const uint8_t BME69X_IDX_S_H_MSB = 23;
static const uint8_t BME69X_IDX_S_H_LSB = 24;
static const uint8_t BME69X_IDX_O_H_MSB = 25;
static const uint8_t BME69X_IDX_O_H_LSB = 24;
static const uint8_t BME69X_IDX_TK10H_C = 26;
static const uint8_t BME69X_IDX_par_h4 = 27;
static const uint8_t BME69X_IDX_par_h3 = 28;
static const uint8_t BME69X_IDX_HLIN2_C = 29;
static const uint8_t BME69X_IDX_RO_C = 35;
static const uint8_t BME69X_IDX_TKR_C_MSB = 34;
static const uint8_t BME69X_IDX_TKR_C_LSB = 33;
static const uint8_t BME69X_IDX_T_AMB_COMP = 36;
static const uint8_t BME69X_IDX_RES_HEAT_RANGE = 39;
static const uint8_t BME69X_RHRANGE_MSK = 0x30;
static const uint8_t BME69X_IDX_RES_HEAT_VAL = 37;
static const uint8_t BME69X_IDX_RANGE_SW_ERR = 41;
static const uint8_t BME69X_RSERROR_MSK = 0xF0;
static const uint8_t BME69X_LEN_CONFIG = 5;
static const uint8_t BME69X_REG_CTRL_MEAS = 0x74;
static const uint8_t BME69X_MODE_MSK = 0x03;
static const uint8_t BME69X_SLEEP_MODE = 0;
static const uint16_t BME69X_PERIOD_POLL = 10000;
static const uint8_t BME69X_LEN_CONFIG = 5;
static const uint8_t BME69X_FILTER_SIZE_127 = 7;
static const uint8_t BME69X_OS_16X = 5;
static const uint8_t BME69X_ODR_NONE = 8;
static const uint8_t BME69X_REG_CTRL_GAS_1 = 0x71;
static const uint8_t BME69X_FILTER_MSK = 0x1C;
static const uint8_t BME69X_FILTER_POS = 2;
static const uint8_t BME69X_ODR20_MSK = 0xE0;
static const uint8_t BME69X_ODR20_POS = 5;
static const uint8_t BME69X_ODR3_POS = 7;
static const uint8_t BME69X_OST_MSK = 0xE0;
static const uint8_t BME69X_OST_POS = 5;
static const uint8_t BME69X_OSP_MSK = 0x1C;
static const uint8_t BME69X_OSP_POS = 2;
static const uint8_t BME69X_OSH_MSK = 0x07;
static const uint8_t BME69X_ODR3_MSK = 0x80;
static const uint8_t BME69X_REG_CTRL_GAS_0 = 0x70;
static const uint8_t BME69X_REG_CTRL_GAS_1 = 0x71;
static const uint8_t BME69X_REG_RES_HEAT0 = 0x5A;
static const uint8_t BME69X_REG_GAS_WAIT0 = 0x64;
static const uint16_t MAX_TEMPERATURE = 400;
static const uint8_t BME69X_ENABLE_HEATER = 0x00;
static const uint8_t BME69X_ENABLE_GAS_MEAS = 0x01;
static const uint8_t BME69X_HCTRL_MSK = 0x08;
static const uint8_t BME69X_HCTRL_POS = 3;
static const uint8_t BME69X_NBCONV_MSK = 0x0F;
static const uint8_t BME69X_RUN_GAS_MSK = 0x30;
static const uint8_t BME69X_RUN_GAS_POS = 5;






uint16_t BME69x_Component::concat_bytes_(uint8_t msb, uint8_t lsb) {
  return (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
}

uint8_t BME69x_Component::set_bits_(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t val) {
  return static_cast<uint8_t>((reg & ~mask) | ((val << pos) & mask));
}

uint8_t BME69x_Component::set_bits_pos0_(uint8_t reg, uint8_t mask, uint8_t val) {
  return static_cast<uint8_t>((reg & ~mask) | (val & mask));
}

bool BME69x_Component::read_calibration_data_(void) {
  uint8_t coeff[BME69X_LEN_COEFF_ALL];

  // Coeff 1
  if (!this->read_bytes(BME69X_REG_COEFF1, coeff, BME69X_LEN_COEFF1)) {
    ESP_LOGE(TAG, "Coeff1 read failed");
    return false;
  }
  // Coeff 2
  if (!this->read_bytes(BME69X_REG_COEFF2, coeff + BME69X_LEN_COEFF1, BME69X_LEN_COEFF2)) {
    ESP_LOGE(TAG, "Coeff2 read failed");
    return false;
  }
  // Coeff 3
  if (!this->read_bytes(BME69X_REG_COEFF3, coeff + BME69X_LEN_COEFF1 + BME69X_LEN_COEFF2, BME69X_LEN_COEFF3)) {
    ESP_LOGE(TAG, "Coeff3 read failed");
    return false;
  }

  // Temperature
  this->calibration_.par_t1 = static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_DO_C_MSB], coeff[BME69X_IDX_DO_C_LSB]));
  this->calibration_.par_t2 = static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_DTK1_C_MSB], coeff[BME69X_IDX_DTK1_C_LSB]));
  this->calibration_.par_t3 = static_cast<int8_t>(coeff[BME69X_IDX_DTK2_C]);

  // Pressure
  this->calibration_.par_p5 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_S_C_MSB], coeff[BME69X_IDX_S_C_LSB]));
  this->calibration_.par_p6 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_TK1S_C_MSB], coeff[BME69X_IDX_TK1S_C_LSB]));
  this->calibration_.par_p7 = static_cast<int8_t>(coeff[BME69X_IDX_TK2S_C]);
  this->calibration_.par_p8 = static_cast<int8_t>(coeff[BME69X_IDX_TK3S_C]);

  this->calibration_.par_p1 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_O_C_MSB], coeff[BME69X_IDX_O_C_LSB]));
  this->calibration_.par_p2 = static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_TK10_C_MSB], coeff[BME69X_IDX_TK10_C_LSB]));
  this->calibration_.par_p3 = static_cast<int8_t>(coeff[BME69X_IDX_TK20_C]);
  this->calibration_.par_p4 = static_cast<int8_t>(coeff[BME69X_IDX_TK30_C]);

  this->calibration_.par_p9  = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_NLS_C_MSB], coeff[BME69X_IDX_NLS_C_LSB]));
  this->calibration_.par_p10 = static_cast<int8_t>(coeff[BME69X_IDX_TKNLS_C]);
  this->calibration_.par_p11 = static_cast<int8_t>(coeff[BME69X_IDX_NLS3_C]);

  // Humidity
  int16_t par_h5 = static_cast<int16_t>((static_cast<int16_t>(coeff[BME69X_IDX_S_H_MSB]) << 4) |
                                        (coeff[BME69X_IDX_S_H_LSB] >> 4));
  if (par_h5 > 2047) par_h5 = static_cast<int16_t>(par_h5 - 4096);
  this->calibration_.par_h5 = par_h5;

  int16_t par_h1 = static_cast<int16_t>((static_cast<int16_t>(coeff[BME69X_IDX_O_H_MSB]) << 4) |
                                        (coeff[BME69X_IDX_O_H_LSB] & 0x0F));
  if (par_h1 > 2047) par_h1 = static_cast<int16_t>(par_h1 - 4096);
  this->calibration_.par_h1 = par_h1;

  this->calibration_.par_h2 = static_cast<int8_t>(coeff[BME69X_IDX_TK10H_C]);
  this->calibration_.par_h4 = static_cast<int8_t>(coeff[BME69X_IDX_par_h4]);
  this->calibration_.par_h3 = static_cast<uint8_t>(coeff[BME69X_IDX_par_h3]);
  this->calibration_.par_h6 = static_cast<uint8_t>(coeff[BME69X_IDX_HLIN2_C]);

  // Gas
  this->calibration_.par_g1 = static_cast<int8_t>(coeff[BME69X_IDX_RO_C]);
  this->calibration_.par_g2 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_TKR_C_MSB], coeff[BME69X_IDX_TKR_C_LSB]));
  this->calibration_.par_g3 = static_cast<int8_t>(coeff[BME69X_IDX_T_AMB_COMP]);

  // Other
  this->calibration_.res_heat_range = static_cast<uint8_t>((coeff[BME69X_IDX_RES_HEAT_RANGE] & BME69X_RHRANGE_MSK) >> 4);
  this->calibration_.res_heat_val   = static_cast<int8_t>(coeff[BME69X_IDX_RES_HEAT_VAL]);
  this->calibration_.range_sw_err   = static_cast<int8_t>((coeff[BME69X_IDX_RANGE_SW_ERR] & BME69X_RSERROR_MSK)) / 16;

  return true;
}

bool BME69x_Component::init_bme69x_(void) {

  // Soft reset
  if(!this->write_byte(BME69X_REG_SOFT_RESET, BME69X_SOFT_RESET_CMD)) {
    this->mark_failed("Soft reset failed");
    return false;
  }

  // Get chip id
  uint8_t chip_id = 0;
  if(!this->read_byte(BME69X_REG_CHIP_ID, &chip_id)) {
    this->mark_failed("Read chip ID failed");
    return false;
  }
  ESP_LOGI(TAG, "Chip ID: 0x%02X", chip_id);

  // Read variant id
  int32_t variant_id = 0;
  if(!this->read_byte(BME69X_REG_VARIANT_ID, (uint8_t*)&variant_id)) {
    this->mark_failed("Read variant ID failed");
    return false;
  }
  ESP_LOGI(TAG, "Variant ID: 0x%02X", variant_id);

  // Read calibration data
  if (!this->read_calibration_data_()) {
    this->mark_failed("Read calibration data failed");
    return false;
  }

  return true;
 
}

bool BME69x_Component::set_sensor_configuration_(void) {
  
  // Register data starting from BME69X_REG_CTRL_GAS_1(0x71) up to BME69X_REG_CONFIG(0x75)
  uint8_t reg_array[BME69X_LEN_CONFIG] = { 0x71, 0x72, 0x73, 0x74, 0x75 };

  // Save current mode
  uint8_t ctrl_meas = 0;
  if (!this->read_byte(BME69X_REG_CTRL_MEAS, &ctrl_meas)) {
    this->mark_failed("Read ctrl_meas failed");
    return false;
  }
  const uint8_t prev_mode = ctrl_meas & BME69X_MODE_MSK;  

  // Configuraion only in sleep mode
  if (!this->set_operating_mode_(BME69X_SLEEP_MODE)) {
    return false;
  }

  // Load 0x71..0x75 register block
  uint8_t data_array[BME69X_LEN_CONFIG] = {0};
  if (!this->read_bytes(BME69X_REG_CTRL_GAS_1, data_array, BME69X_LEN_CONFIG)) {
    this->mark_failed("Read config block failed");
    return false;
  }

  const uint8_t os_t = this->temperature_oversampling_;
  const uint8_t os_p = this->pressure_oversampling_;
  const uint8_t os_h = this->humidity_oversampling_;
  const uint8_t filt = this->iir_filter_;

  // ODR: NONE -> odr20 = 0, odr3 = 1
  const uint8_t odr20 = 0;
  const uint8_t odr3  = 1;

  // CONFIG (0x75): Filter and ODR20
  data_array[4] = set_bits_(data_array[4], BME69X_FILTER_MSK, BME69X_FILTER_POS, filt);
  data_array[4] = set_bits_(data_array[4], BME69X_ODR20_MSK,  BME69X_ODR20_POS,  odr20);

  // CTRL_MEAS (0x74): OS temp + OS pres
  data_array[3] = set_bits_(data_array[3], BME69X_OST_MSK, BME69X_OST_POS, os_t);
  data_array[3] = set_bits_(data_array[3], BME69X_OSP_MSK, BME69X_OSP_POS, os_p);

  // CTRL_HUM (0x72): OS hum
  data_array[1] = set_bits_pos0_(data_array[1], BME69X_OSH_MSK, os_h);

  // CTRL_GAS_1 (0x71): ODR3
  data_array[0] = set_bits_(data_array[0], BME69X_ODR3_MSK, BME69X_ODR3_POS, odr3);

  // Write the block back 0x71..0x75
  for (uint8_t i = 0; i < BME69X_LEN_CONFIG; i++) {
    const uint8_t reg = static_cast<uint8_t>(BME69X_REG_CTRL_GAS_1 + i);
    if (!this->write_byte(reg, data_array[i])) {
      this->mark_failed("Write config block failed");
      return false;
    }
  }

  // Restore previous mode if not sleep
  if (prev_mode != BME69X_SLEEP_MODE) {
    if (!this->set_operating_mode_(prev_mode)) {
      this->mark_failed("Restore op mode failed");
      return false;
    }
  }  

  return true;

}

bool BME69x_Component::set_heater_configuration_(void) {

  // Set to sleep mode
  if (!this->set_operating_mode_(BME69X_SLEEP_MODE)) {
    return false;
  }

  const uint8_t heater_resistance = calculate_heater_resistance_(this->heater_temperature_);
  const uint8_t gas_wait = calculate_gas_wait_(this->heater_duration_);

  // Write heater resistance
  if (!this->write_byte(BME69X_REG_RES_HEAT0, heater_resistance)) {
    this->mark_failed("Write heater resistance failed");
    return false;
  }

  // Write gas wait time
  if (!this->write_byte(BME69X_REG_GAS_WAIT0, gas_wait)) {
    this->mark_failed("Write gas wait failed");
    return false;
  }

  uint8_t heater_ctrl = 0;
  uint8_t gas_ctrl = 0;

  uint8_t ctrl_gas_data[2] = {0};

  if (!this->read_bytes(BME69X_REG_CTRL_GAS_0, ctrl_gas_data, 2)) {
    this->mark_failed("Read heater ctrl failed");
    return false;
  }

  ctrl_gas_data[0] = set_bits_(ctrl_gas_data[0], BME69X_HCTRL_MSK, BME69X_HCTRL_POS, BME69X_ENABLE_HEATER);
  ctrl_gas_data[1] = set_bits_pos0_(ctrl_gas_data[1], BME69X_NBCONV_MSK, 0);
  ctrl_gas_data[1] = set_bits_(ctrl_gas_data[1], BME69X_RUN_GAS_MSK, BME69X_RUN_GAS_POS, BME69X_ENABLE_GAS_MEAS);

  if(!this->write_bytes(BME69X_REG_CTRL_GAS_0, ctrl_gas_data, 2)) {
    this->mark_failed("Write heater ctrl failed");
    return false;
  }

  return true;

}

uint8_t BME69x_Component::calculate_heater_resistance_(uint16_t temperature) {

  if (temperature > MAX_TEMPERATURE) 
  {
      temperature = MAX_TEMPERATURE;
  }

  int32_t var1 = (((int32_t)this->ambient_temperature_ * this->calibration_.par_g3) / 1000U) * 256; 
  int32_t var2 = (this->calibration_.par_g1 + 784) * (((((this->calibration_.par_g2 + 154009UL) * temperature * 5) / 100) + 3276800ULL) / 10);
  int32_t var3 = var1 + (var2 >> 1);
  int32_t var4 = (var3 / (this->calibration_.res_heat_range + 4));
  int32_t var5 = (131 * this->calibration_.res_heat_val) + 65536UL;
  int32_t heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);

  return static_cast<uint8_t>((heatr_res_x100 + 50) / 100);

}

uint8_t BME69x_Component::calculate_gas_wait_(uint16_t duration) {

  uint8_t factor = 0;
  uint8_t durval;

  if (duration >= 0xfc0)
  {
      durval = 0xff; /* Max duration*/
  }
  else
  {
      while (duration > 0x3F)
      {
          duration = duration / 4;
          factor += 1;
      }

      durval = (uint8_t)(duration + (factor * 64));
  }

  return durval;

}

bool BME69x_Component::set_operating_mode_(uint8_t target_op_mode) {
  uint8_t tmp_current_mode = 0;
  uint8_t current_mode = 0;

  const uint32_t max_tries = 20;  // Timeout
  uint32_t tries = 0;

  do {
    if (!this->read_byte(BME69X_REG_CTRL_MEAS, &tmp_current_mode)) {
      this->mark_failed("Read ctrl_meas failed");
      return false;
    }

    current_mode = tmp_current_mode & BME69X_MODE_MSK;

    if (current_mode != BME69X_SLEEP_MODE) {
      tmp_current_mode &= static_cast<uint8_t>(~BME69X_MODE_MSK);  // sleep
      if (!this->write_byte(BME69X_REG_CTRL_MEAS, tmp_current_mode)) {
        this->mark_failed("Set sleep mode failed");
        return false;
      }
      delayMicroseconds(BME69X_PERIOD_POLL);
    }

    if (++tries >= max_tries) {
      this->mark_failed("Sleep poll timeout");
      return false;
    }

  } while (current_mode != BME69X_SLEEP_MODE);

  if (target_op_mode != BME69X_SLEEP_MODE) {
    tmp_current_mode =
        static_cast<uint8_t>((tmp_current_mode & static_cast<uint8_t>(~BME69X_MODE_MSK)) |
                             (target_op_mode & BME69X_MODE_MSK));
    if (!this->write_byte(BME69X_REG_CTRL_MEAS, tmp_current_mode)) {
      this->mark_failed("Set operating mode failed");
      return false;
    }
    delayMicroseconds(BME69X_PERIOD_POLL);
  }

  return true;
}

void BME69x_Component::setup() {

  if (!this->init_bme69x_()) {
    return;
  }

  if (!this->set_sensor_configuration_()) {
    return;
  }

  if (!this->set_heater_configuration_()) {
    return;
  }


  // We do an initial read to populate the sensor values right away
  this->update();
}

void BME680Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BME680:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  ESP_LOGCONFIG(TAG, "  IIR Filter: %s", iir_filter_to_str(this->iir_filter_));
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->temperature_oversampling_));
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->pressure_oversampling_));
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->humidity_oversampling_));
  LOG_SENSOR("  ", "Gas Resistance", this->gas_resistance_sensor_);
  if (this->heater_duration_ == 0 || this->heater_temperature_ == 0) {
    ESP_LOGCONFIG(TAG, "  Heater OFF");
  } else {
    ESP_LOGCONFIG(TAG, "  Heater temperature=%u°C duration=%ums", this->heater_temperature_, this->heater_duration_);
  }
}

float BME680Component::get_setup_priority() const { return setup_priority::DATA; }

void BME680Component::update() {
  uint8_t meas_control = 0;  // No need to fetch, we're setting all fields
  meas_control |= (this->temperature_oversampling_ & 0b111) << 5;
  meas_control |= (this->pressure_oversampling_ & 0b111) << 2;
  meas_control |= 0b01;  // forced mode
  if (!this->write_byte(BME680_REGISTER_CONTROL_MEAS, meas_control)) {
    this->status_set_warning();
    return;
  }

  this->set_timeout("data", this->calc_meas_duration_(), [this]() { this->read_data_(); });
}

uint8_t BME680Component::calc_heater_resistance_(uint16_t temperature) {
  if (temperature < 200)
    temperature = 200;
  if (temperature > 400)
    temperature = 400;

  const int8_t ambient_temperature = this->calibration_.ambient_temperature;
  const int8_t gh1 = this->calibration_.gh1;
  const int16_t gh2 = this->calibration_.gh2;
  const int8_t gh3 = this->calibration_.gh3;
  const uint8_t res_heat_range = this->calibration_.res_heat_range;
  const int8_t res_heat_val = this->calibration_.res_heat_val;

  uint8_t heatr_res;
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  int32_t var5;
  int32_t heatr_res_x100;

  var1 = (((int32_t) ambient_temperature * gh3) / 1000) * 256;
  var2 = (gh1 + 784) * (((((gh2 + 154009) * temperature * 5) / 100) + 3276800) / 10);
  var3 = var1 + (var2 / 2);
  var4 = (var3 / (res_heat_range + 4));
  var5 = (131 * res_heat_val) + 65536;
  heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
  heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

  return heatr_res;
}
uint8_t BME680Component::calc_heater_duration_(uint16_t duration) {
  uint8_t factor = 0;
  uint8_t duration_value;

  if (duration >= 0xfc0) {
    duration_value = 0xff;
  } else {
    while (duration > 0x3F) {
      duration /= 4;
      factor += 1;
    }
    duration_value = duration + (factor * 64);
  }

  return duration_value;
}
void BME680Component::read_data_() {
  uint8_t data[15];
  if (!this->read_bytes(BME680_REGISTER_FIELD0, data, 15)) {
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(NAN);
    if (this->pressure_sensor_ != nullptr)
      this->pressure_sensor_->publish_state(NAN);
    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(NAN);
    if (this->gas_resistance_sensor_ != nullptr)
      this->gas_resistance_sensor_->publish_state(NAN);
    ESP_LOGW(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();

  uint32_t raw_temperature = (uint32_t(data[5]) << 12) | (uint32_t(data[6]) << 4) | (uint32_t(data[7]) >> 4);
  uint32_t raw_pressure = (uint32_t(data[2]) << 12) | (uint32_t(data[3]) << 4) | (uint32_t(data[4]) >> 4);
  uint32_t raw_humidity = (uint32_t(data[8]) << 8) | uint32_t(data[9]);
  uint16_t raw_gas = (uint16_t) ((uint32_t) data[13] * 4 | (((uint32_t) data[14]) / 64));
  uint8_t gas_range = data[14] & 0x0F;

  float temperature = this->calc_temperature_(raw_temperature);
  float pressure = this->calc_pressure_(raw_pressure);
  float humidity = this->calc_humidity_(raw_humidity);
  float gas_resistance = this->calc_gas_resistance_(raw_gas, gas_range);

  bool gas_valid = (data[14] >> 5) & 1;
  bool heat_stable = (data[14] >> 4) & 1;
  if (this->heater_temperature_ == 0 || this->heater_duration_ == 0)
    heat_stable = true;  // Allow reporting gas resistance when heater is disabled

  ESP_LOGD(TAG, "Got temperature=%.1f°C pressure=%.1fhPa humidity=%.1f%% gas_resistance=%.1fΩ", temperature, pressure,
           humidity, gas_resistance);
  if (!gas_valid)
    ESP_LOGW(TAG, "Gas measurement unsuccessful, reading invalid!");
  if (!heat_stable)
    ESP_LOGW(TAG, "Heater unstable, reading invalid! (Normal for a few readings after a power cycle)");

  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);
  if (this->pressure_sensor_ != nullptr)
    this->pressure_sensor_->publish_state(pressure);
  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(humidity);
  if (this->gas_resistance_sensor_ != nullptr) {
    if (gas_valid && heat_stable) {
      this->gas_resistance_sensor_->publish_state(gas_resistance);
    } else {
      this->status_set_warning();
      this->gas_resistance_sensor_->publish_state(NAN);
    }
  }
}

float BME680Component::calc_temperature_(uint32_t raw_temperature) {
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float calc_temp = 0;
  float temp_adc = raw_temperature;

  const float t1 = this->calibration_.t1;
  const float t2 = this->calibration_.t2;
  const float t3 = this->calibration_.t3;

  /* calculate var1 data */
  var1 = ((temp_adc / 16384.0f) - (t1 / 1024.0f)) * t2;

  /* calculate var2 data */
  var3 = (temp_adc / 131072.0f) - (t1 / 8192.0f);
  var2 = var3 * var3 * t3 * 16.0f;

  /* t_fine value*/
  this->calibration_.tfine = (var1 + var2);

  /* compensated temperature data*/
  calc_temp = ((this->calibration_.tfine) / 5120.0f);

  return calc_temp;
}
float BME680Component::calc_pressure_(uint32_t raw_pressure) {
  const float tfine = this->calibration_.tfine;
  const float p1 = this->calibration_.p1;
  const float p2 = this->calibration_.p2;
  const float p3 = this->calibration_.p3;
  const float p4 = this->calibration_.p4;
  const float p5 = this->calibration_.p5;
  const float p6 = this->calibration_.p6;
  const float p7 = this->calibration_.p7;
  const float p8 = this->calibration_.p8;
  const float p9 = this->calibration_.p9;
  const float p10 = this->calibration_.p10;

  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float var4 = 0;
  float calc_pres = 0;

  var1 = (tfine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * (p6 / 131072.0f);
  var2 = var2 + var1 * p5 * 2.0f;
  var2 = (var2 / 4.0f) + (p4 * 65536.0f);
  var1 = (((p3 * var1 * var1) / 16384.0f) + (p2 * var1)) / 524288.0f;
  var1 = (1.0f + (var1 / 32768.0f)) * p1;
  calc_pres = 1048576.0f - float(raw_pressure);

  /* Avoid exception caused by division by zero */
  if (int(var1) != 0) {
    calc_pres = ((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = (p9 * calc_pres * calc_pres) / 2147483648.0f;
    var2 = calc_pres * (p8 / 32768.0f);
    var4 = calc_pres / 256.0f;
    var3 = var4 * var4 * var4 * (p10 / 131072.0f);
    calc_pres = calc_pres + (var1 + var2 + var3 + (p7 * 128.0f)) / 16.0f;
  } else {
    calc_pres = 0;
  }

  return calc_pres / 100.0f;
}

float BME680Component::calc_humidity_(uint16_t raw_humidity) {
  const float tfine = this->calibration_.tfine;
  const float h1 = this->calibration_.h1;
  const float h2 = this->calibration_.h2;
  const float h3 = this->calibration_.h3;
  const float h4 = this->calibration_.h4;
  const float h5 = this->calibration_.h5;
  const float h6 = this->calibration_.h6;
  const float h7 = this->calibration_.h7;

  float calc_hum = 0;
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float var4 = 0;
  float temp_comp;

  /* compensated temperature data*/
  temp_comp = tfine / 5120.0f;

  var1 = float(raw_humidity) - (h1 * 16.0f + ((h3 / 2.0f) * temp_comp));
  var2 = var1 *
         (((h2 / 262144.0f) * (1.0f + ((h4 / 16384.0f) * temp_comp) + ((h5 / 1048576.0f) * temp_comp * temp_comp))));
  var3 = h6 / 16384.0f;
  var4 = h7 / 2097152.0f;

  calc_hum = var2 + (var3 + var4 * temp_comp) * var2 * var2;

  if (calc_hum > 100.0f) {
    calc_hum = 100.0f;
  } else if (calc_hum < 0.0f) {
    calc_hum = 0.0f;
  }

  return calc_hum;
}
float BME680Component::calc_gas_resistance_(uint16_t raw_gas, uint8_t range) {
  float calc_gas_res;
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float raw_gas_f = raw_gas;
  float range_f = 1U << range;
  const float range_sw_err = this->calibration_.range_sw_err;

  var1 = 1340.0f + (5.0f * range_sw_err);
  var2 = var1 * (1.0f + BME680_GAS_LOOKUP_TABLE_1[range] / 100.0f);
  var3 = 1.0f + (BME680_GAS_LOOKUP_TABLE_2[range] / 100.0f);

  calc_gas_res = 1.0f / (var3 * 0.000000125f * range_f * (((raw_gas_f - 512.0f) / var2) + 1.0f));

  return calc_gas_res;
}
uint32_t BME680Component::calc_meas_duration_() {
  uint32_t tph_dur;  // Calculate in us
  uint32_t meas_cycles;
  const uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};

  meas_cycles = os_to_meas_cycles[this->temperature_oversampling_];
  meas_cycles += os_to_meas_cycles[this->pressure_oversampling_];
  meas_cycles += os_to_meas_cycles[this->humidity_oversampling_];

  /* TPH measurement duration */
  tph_dur = meas_cycles * 1963u;
  tph_dur += 477 * 4;  // TPH switching duration
  tph_dur += 477 * 5;  // Gas measurement duration
  tph_dur += 500;      // Get it to the closest whole number.
  tph_dur /= 1000;     // Convert to ms

  tph_dur += 1;  // Wake up duration of 1ms

  /* The remaining time should be used for heating */
  tph_dur += this->heater_duration_;

  return tph_dur;
}
void BME680Component::set_temperature_oversampling(BME680Oversampling temperature_oversampling) {
  this->temperature_oversampling_ = temperature_oversampling;
}
void BME680Component::set_pressure_oversampling(BME680Oversampling pressure_oversampling) {
  this->pressure_oversampling_ = pressure_oversampling;
}
void BME680Component::set_humidity_oversampling(BME680Oversampling humidity_oversampling) {
  this->humidity_oversampling_ = humidity_oversampling;
}
void BME680Component::set_iir_filter(BME680IIRFilter iir_filter) { this->iir_filter_ = iir_filter; }
void BME680Component::set_heater(uint16_t heater_temperature, uint16_t heater_duration) {
  this->heater_temperature_ = heater_temperature;
  this->heater_duration_ = heater_duration;
}

}  // namespace bme690
}  // namespace esphome
