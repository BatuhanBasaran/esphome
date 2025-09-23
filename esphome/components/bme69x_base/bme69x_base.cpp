#include "bme69x_base.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bme69x_base {

static const char *const TAG = "bme69x.sensor";

static const uint8_t AMBIENT_TEMPERATURE = 25;
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
static const uint8_t BME69X_DISABLE_HEATER = 0x01;
static const uint8_t BME69X_ENABLE_GAS_MEAS = 0x01;
static const uint8_t BME69X_DISABLE_GAS_MEAS = 0x00;
static const uint8_t BME69X_HCTRL_MSK = 0x08;
static const uint8_t BME69X_HCTRL_POS = 3;
static const uint8_t BME69X_NBCONV_MSK = 0x0F;
static const uint8_t BME69X_RUN_GAS_MSK = 0x30;
static const uint8_t BME69X_RUN_GAS_POS = 5;
static const uint8_t BME69X_FORCED_MODE = 1;
static const uint8_t BME69X_LEN_FIELD = 17;
static const uint8_t BME69X_REG_FIELD0 = 0x1D;
static const uint8_t BME69X_NEW_DATA_MSK = 0x80;
static const uint8_t BME69X_GAS_INDEX_MSK = 0x0F;
static const uint8_t BME69X_GAS_RANGE_MSK = 0x0F;
static const uint8_t BME69X_GASM_VALID_MSK = 0x20;
static const uint8_t BME69X_HEAT_STAB_MSK = 0x10;
static const uint8_t BME69X_REG_IDAC_HEAT0 = 0x50;

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
  this->calibration_.par_t1 =
      static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_DO_C_MSB], coeff[BME69X_IDX_DO_C_LSB]));
  this->calibration_.par_t2 =
      static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_DTK1_C_MSB], coeff[BME69X_IDX_DTK1_C_LSB]));
  this->calibration_.par_t3 = static_cast<int8_t>(coeff[BME69X_IDX_DTK2_C]);

  // Pressure
  this->calibration_.par_p5 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_S_C_MSB], coeff[BME69X_IDX_S_C_LSB]));
  this->calibration_.par_p6 =
      static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_TK1S_C_MSB], coeff[BME69X_IDX_TK1S_C_LSB]));
  this->calibration_.par_p7 = static_cast<int8_t>(coeff[BME69X_IDX_TK2S_C]);
  this->calibration_.par_p8 = static_cast<int8_t>(coeff[BME69X_IDX_TK3S_C]);

  this->calibration_.par_p1 = static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_O_C_MSB], coeff[BME69X_IDX_O_C_LSB]));
  this->calibration_.par_p2 =
      static_cast<uint16_t>(concat_bytes_(coeff[BME69X_IDX_TK10_C_MSB], coeff[BME69X_IDX_TK10_C_LSB]));
  this->calibration_.par_p3 = static_cast<int8_t>(coeff[BME69X_IDX_TK20_C]);
  this->calibration_.par_p4 = static_cast<int8_t>(coeff[BME69X_IDX_TK30_C]);

  this->calibration_.par_p9 =
      static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_NLS_C_MSB], coeff[BME69X_IDX_NLS_C_LSB]));
  this->calibration_.par_p10 = static_cast<int8_t>(coeff[BME69X_IDX_TKNLS_C]);
  this->calibration_.par_p11 = static_cast<int8_t>(coeff[BME69X_IDX_NLS3_C]);

  // Humidity
  int16_t par_h5 =
      static_cast<int16_t>((static_cast<int16_t>(coeff[BME69X_IDX_S_H_MSB]) << 4) | (coeff[BME69X_IDX_S_H_LSB] >> 4));
  if (par_h5 > 2047)
    par_h5 = static_cast<int16_t>(par_h5 - 4096);
  this->calibration_.par_h5 = par_h5;

  int16_t par_h1 =
      static_cast<int16_t>((static_cast<int16_t>(coeff[BME69X_IDX_O_H_MSB]) << 4) | (coeff[BME69X_IDX_O_H_LSB] & 0x0F));
  if (par_h1 > 2047)
    par_h1 = static_cast<int16_t>(par_h1 - 4096);
  this->calibration_.par_h1 = par_h1;

  this->calibration_.par_h2 = static_cast<int8_t>(coeff[BME69X_IDX_TK10H_C]);
  this->calibration_.par_h4 = static_cast<int8_t>(coeff[BME69X_IDX_par_h4]);
  this->calibration_.par_h3 = static_cast<uint8_t>(coeff[BME69X_IDX_par_h3]);
  this->calibration_.par_h6 = static_cast<uint8_t>(coeff[BME69X_IDX_HLIN2_C]);

  // Gas
  this->calibration_.par_g1 = static_cast<int8_t>(coeff[BME69X_IDX_RO_C]);
  this->calibration_.par_g2 =
      static_cast<int16_t>(concat_bytes_(coeff[BME69X_IDX_TKR_C_MSB], coeff[BME69X_IDX_TKR_C_LSB]));
  this->calibration_.par_g3 = static_cast<int8_t>(coeff[BME69X_IDX_T_AMB_COMP]);

  // Other
  this->calibration_.res_heat_range =
      static_cast<uint8_t>((coeff[BME69X_IDX_RES_HEAT_RANGE] & BME69X_RHRANGE_MSK) >> 4);
  this->calibration_.res_heat_val = static_cast<int8_t>(coeff[BME69X_IDX_RES_HEAT_VAL]);
  this->calibration_.range_sw_err = static_cast<int8_t>((coeff[BME69X_IDX_RANGE_SW_ERR] & BME69X_RSERROR_MSK)) / 16;

  return true;
}

bool BME69x_Component::init_bme69x_(void) {
  // Soft reset
  if (!this->write_byte(BME69X_REG_SOFT_RESET, BME69X_SOFT_RESET_CMD)) {
    this->mark_failed("Soft reset failed");
    return false;
  }

  // Get chip id
  uint8_t chip_id = 0;
  if (!this->read_byte(BME69X_REG_CHIP_ID, &chip_id)) {
    this->mark_failed("Read chip ID failed");
    return false;
  }
  ESP_LOGI(TAG, "Chip ID: 0x%02X", chip_id);

  // Read variant id
  uint8_t variant_id = 0;
  if (!this->read_byte(BME69X_REG_VARIANT_ID, &variant_id)) {
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
  uint8_t reg_array[BME69X_LEN_CONFIG] = {0x71, 0x72, 0x73, 0x74, 0x75};

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
  const uint8_t odr3 = 1;

  // CONFIG (0x75): Filter and ODR20
  data_array[4] = set_bits_(data_array[4], BME69X_FILTER_MSK, BME69X_FILTER_POS, filt);
  data_array[4] = set_bits_(data_array[4], BME69X_ODR20_MSK, BME69X_ODR20_POS, odr20);

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

  uint8_t ctrl_gas_data[2] = {0};

  if (!this->read_bytes(BME69X_REG_CTRL_GAS_0, ctrl_gas_data, 2)) {
    this->mark_failed("Read heater ctrl failed");
    return false;
  }

  uint8_t heater_enabled = (this->heater_temperature_ > 0 && this->heater_duration_ > 0) ? 1 : 0;
  uint8_t heater_ctrl = 0;
  uint8_t gas_ctrl = 0;

  if (heater_enabled) {
    heater_ctrl = BME69X_ENABLE_HEATER;
    gas_ctrl = BME69X_ENABLE_GAS_MEAS;
  } else {
    heater_ctrl = BME69X_DISABLE_HEATER;
    gas_ctrl = BME69X_DISABLE_GAS_MEAS;
  }

  ctrl_gas_data[0] = set_bits_(ctrl_gas_data[0], BME69X_HCTRL_MSK, BME69X_HCTRL_POS, heater_ctrl);
  ctrl_gas_data[1] = set_bits_pos0_(ctrl_gas_data[1], BME69X_NBCONV_MSK, 0);
  ctrl_gas_data[1] = set_bits_(ctrl_gas_data[1], BME69X_RUN_GAS_MSK, BME69X_RUN_GAS_POS, gas_ctrl);

  if (!this->write_bytes(BME69X_REG_CTRL_GAS_0, ctrl_gas_data, 2)) {
    this->mark_failed("Write heater ctrl failed");
    return false;
  }

  return true;
}

uint8_t BME69x_Component::calculate_heater_resistance_(uint16_t temperature) {
  if (temperature > MAX_TEMPERATURE) {
    temperature = MAX_TEMPERATURE;
  }

  int32_t var1 = (((int32_t) AMBIENT_TEMPERATURE * this->calibration_.par_g3) / 1000U) * 256;
  int32_t var2 = (this->calibration_.par_g1 + 784) *
                 (((((this->calibration_.par_g2 + 154009UL) * temperature * 5) / 100) + 3276800ULL) / 10);
  int32_t var3 = var1 + (var2 >> 1);
  int32_t var4 = (var3 / (this->calibration_.res_heat_range + 4));
  int32_t var5 = (131 * this->calibration_.res_heat_val) + 65536UL;
  int32_t heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);

  return static_cast<uint8_t>((heatr_res_x100 + 50) / 100);
}

uint8_t BME69x_Component::calculate_gas_wait_(uint16_t duration) {
  uint8_t factor = 0;
  uint8_t durval;

  if (duration >= 0xfc0) {
    durval = 0xff; /* Max duration*/
  } else {
    while (duration > 0x3F) {
      duration = duration / 4;
      factor += 1;
    }

    durval = (uint8_t) (duration + (factor * 64));
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
    tmp_current_mode = static_cast<uint8_t>((tmp_current_mode & static_cast<uint8_t>(~BME69X_MODE_MSK)) |
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
    this->mark_failed("Init failed");
    return;
  }

  ESP_LOGV(TAG, "Sensor initialized");

  if (!this->set_sensor_configuration_()) {
    this->mark_failed("Set sensor configuration failed");
    return;
  }

  ESP_LOGV(TAG, "Sensor configured");

  if (!this->set_heater_configuration_()) {
    this->mark_failed("Set heater configuration failed");
    return;
  }

  ESP_LOGV(TAG, "Heater configured");
}

void BME69x_Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BME69x:");
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

void BME69x_Component::update() {
  // Set to forced mode
  if (!this->set_operating_mode_(BME69X_FORCED_MODE)) {
    this->mark_failed("Set forced mode failed");
    return;
  }

  uint32_t current_time = micros();

  if (!get_start_time_) {
    get_start_time_ = true;
    start_time_ = micros();
    measure_duration_ = this->calculate_measure_duration_();
  }

  if ((current_time - start_time_) > measure_duration_) {
    float temperature, pressure, humidity, gas_resistance;

    if (!this->get_sensor_data_(&temperature, &pressure, &humidity, &gas_resistance)) {
      ESP_LOGW(TAG, "Get sensor data failed");
      return;
    }

    if (this->temperature_sensor_ != nullptr) {
      this->temperature_sensor_->publish_state(temperature);
    }
    if (this->pressure_sensor_ != nullptr) {
      this->pressure_sensor_->publish_state(pressure);
    }
    if (this->humidity_sensor_ != nullptr) {
      this->humidity_sensor_->publish_state(humidity);
    }
    if (this->gas_resistance_sensor_ != nullptr) {
      this->gas_resistance_sensor_->publish_state(gas_resistance);
    }

    get_start_time_ = false;
  }
}

bool BME69x_Component::get_sensor_data_(float *temperature, float *pressure, float *humidity, float *gas_resistance) {
  uint8_t buffer[BME69X_LEN_FIELD] = {0};
  uint8_t tries = 5;

  while (tries--) {
    if (!this->read_bytes(BME69X_REG_FIELD0, buffer, BME69X_LEN_FIELD)) {
      this->mark_failed("Read sensor data failed");
      return false;
    }

    uint8_t status = buffer[0] & BME69X_NEW_DATA_MSK;
    uint8_t gas_index = buffer[0] & BME69X_GAS_INDEX_MSK;
    uint8_t gas_range = buffer[16] & BME69X_GAS_RANGE_MSK;

    status |= (buffer[16] & BME69X_GASM_VALID_MSK);
    status |= (buffer[16] & BME69X_HEAT_STAB_MSK);

    uint32_t adc_pres = (uint32_t(buffer[2]) << 16) | (uint32_t(buffer[3]) << 8) | uint32_t(buffer[4]);
    uint32_t adc_temp = (uint32_t(buffer[5]) << 16) | (uint32_t(buffer[6]) << 8) | uint32_t(buffer[7]);
    uint16_t adc_hum = (uint16_t(buffer[8]) << 8) | uint16_t(buffer[9]);
    uint16_t adc_gas = (uint16_t(buffer[15]) << 2) | uint16_t(buffer[16] >> 6);

    if ((status & BME69X_NEW_DATA_MSK) != 0) {
      uint8_t heater_res, idac, gas_wait;

      if (!this->read_byte(BME69X_REG_RES_HEAT0 + gas_index, &heater_res)) {
        this->mark_failed("Read heater resistance failed");
        return false;
      }
      if (!this->read_byte(BME69X_REG_IDAC_HEAT0 + gas_index, &idac)) {
        this->mark_failed("Read idac failed");
        return false;
      }
      if (!this->read_byte(BME69X_REG_GAS_WAIT0 + gas_index, &gas_wait)) {
        this->mark_failed("Read gas wait failed");
        return false;
      }

      *temperature = this->calculate_temperature_(adc_temp);
      *pressure = this->calculate_pressure_(adc_pres, temperature);
      *humidity = this->calc_humidity_(adc_hum, temperature);
      *gas_resistance = this->calc_gas_resistance_(adc_gas, gas_range);
      return true;
    }

    delayMicroseconds(BME69X_PERIOD_POLL);
  }

  return false;
}

float BME69x_Component::calc_gas_resistance_(uint16_t raw_gas, uint8_t range) {
  const uint32_t var1 = static_cast<uint32_t>(262144U) >> range;
  int32_t var2 = static_cast<int32_t>(raw_gas) - 512;

  var2 *= 3;
  var2 += 4096;

  return 1000000.0f * static_cast<float>(var1) / static_cast<float>(var2);
}

float BME69x_Component::calc_humidity_(uint16_t raw_humidity, float *comp_temperature) {
  if (!comp_temperature)
    return NAN;

  const auto &c = this->calibration_;
  const double T = static_cast<double>(*comp_temperature);
  const double temp_comp = (T * 5120.0) - 76800.0;

  const double oh = static_cast<double>(c.par_h1) * static_cast<double>(1ULL << 6);
  const double sh = static_cast<double>(c.par_h5) / static_cast<double>(1ULL << 16);
  const double tk10h = static_cast<double>(c.par_h2) / static_cast<double>(1ULL << 14);
  const double tk1sh = static_cast<double>(c.par_h4) / static_cast<double>(1ULL << 26);
  const double tk2sh = static_cast<double>(c.par_h3) / static_cast<double>(1ULL << 26);
  const double hlin2 = static_cast<double>(c.par_h6) / static_cast<double>(1ULL << 19);

  const double adc = static_cast<double>(raw_humidity);

  const double hoff = adc - (oh + tk10h * temp_comp);
  const double hsens = hoff * sh * (1.0 + (tk1sh * temp_comp) + (tk1sh * tk2sh * temp_comp * temp_comp));
  double hum = hsens * (1.0 - hlin2 * hsens);

  const int32_t hum_int = static_cast<int32_t>(hum * 1000.0);
  if (hum_int >= 100000)
    hum = 100.0;
  else if (hum_int < 0)
    hum = 0.0;

  return static_cast<float>(hum);
}

float BME69x_Component::calculate_pressure_(uint32_t raw_pressure, float *comp_temperature) {
  if (comp_temperature == nullptr)
    return NAN;

  const auto &c = this->calibration_;

  const uint32_t o = static_cast<uint32_t>(c.par_p1) * static_cast<uint32_t>(1ULL << 3);
  const double tk10 = static_cast<double>(c.par_p2) / static_cast<double>(1ULL << 6);
  const double tk20 = static_cast<double>(c.par_p3) / static_cast<double>(1ULL << 8);
  const double tk30 = static_cast<double>(c.par_p4) / static_cast<double>(1ULL << 15);

  const double s = (static_cast<double>(c.par_p5) - static_cast<double>(1ULL << 14)) / static_cast<double>(1ULL << 20);
  const double tk1s =
      (static_cast<double>(c.par_p6) - static_cast<double>(1ULL << 14)) / static_cast<double>(1ULL << 29);
  const double tk2s = static_cast<double>(c.par_p7) / static_cast<double>(1ULL << 32);
  const double tk3s = static_cast<double>(c.par_p8) / static_cast<double>(1ULL << 37);

  const double nls = static_cast<double>(c.par_p9) / static_cast<double>(1ULL << 48);
  const double tknls = static_cast<double>(c.par_p10) / static_cast<double>(1ULL << 48);

  /*
   * NLS3 = par_p11 / 2^65
   * 2^65 is exceeding the width of 'double' datatype and hence we splitted into two factors since A^(x+y) = A^x * A^y
   */
  const double nls3 =
      static_cast<double>(c.par_p11) / (static_cast<double>(1ULL << 35) * static_cast<double>(1ULL << 30));

  const double T = static_cast<double>(*comp_temperature);
  const double P = static_cast<double>(raw_pressure);

  const double tmp1 = static_cast<double>(o) + (tk10 * T) + (tk20 * T * T) + (tk30 * T * T * T);

  const double tmp2 = P * (s + (tk1s * T) + (tk2s * T * T) + (tk3s * T * T * T));
  const double tmp3 = P * P * (nls + (tknls * T));
  const double tmp4 = P * P * P * nls3;

  const double calc_pres = tmp1 + tmp2 + tmp3 + tmp4;

  return static_cast<float>(calc_pres);
}

float BME69x_Component::calculate_temperature_(uint32_t raw_temperature) {
  const int32_t do1 = static_cast<int32_t>(this->calibration_.par_t1) << 8;
  const double dtk1 = static_cast<double>(this->calibration_.par_t2) / static_cast<double>(1ULL << 30);
  const double dtk2 = static_cast<double>(this->calibration_.par_t3) / static_cast<double>(1ULL << 48);
  const int32_t cf = static_cast<int32_t>(raw_temperature) - do1;

  const double temp1 = static_cast<double>(cf) * dtk1;
  const double temp2 = static_cast<double>(cf) * static_cast<double>(cf) * dtk2;

  return static_cast<float>(temp1 + temp2);
}

uint32_t BME69x_Component::calculate_measure_duration_(void) {
  uint32_t measure_cycles = oversampling_to_measurement_cycles_(this->temperature_oversampling_) +
                            oversampling_to_measurement_cycles_(this->pressure_oversampling_) +
                            oversampling_to_measurement_cycles_(this->humidity_oversampling_);

  // TPH measurement duration
  uint32_t measure_duration = measure_cycles * 1963;

  // TPH switching duration
  measure_duration += 477 * 4;

  // Gas measurement duration
  measure_duration += 477 * 5;

  // Add 1ms margin
  measure_duration += 1000;

  return measure_duration;
}

uint8_t BME69x_Component::oversampling_to_measurement_cycles_(BME69x_Oversampling oversampling) {
  switch (oversampling) {
    case BME69x_OVERSAMPLING_NONE:
      return 0;
    case BME69x_OVERSAMPLING_1X:
      return 1;
    case BME69x_OVERSAMPLING_2X:
      return 2;
    case BME69x_OVERSAMPLING_4X:
      return 4;
    case BME69x_OVERSAMPLING_8X:
      return 8;
    case BME69x_OVERSAMPLING_16X:
      return 16;
    default:
      return 0;
  }
}

static const char *oversampling_to_str(BME69x_Oversampling oversampling) {
  switch (oversampling) {
    case BME69x_OVERSAMPLING_NONE:
      return "None";
    case BME69x_OVERSAMPLING_1X:
      return "1x";
    case BME69x_OVERSAMPLING_2X:
      return "2x";
    case BME69x_OVERSAMPLING_4X:
      return "4x";
    case BME69x_OVERSAMPLING_8X:
      return "8x";
    case BME69x_OVERSAMPLING_16X:
      return "16x";
    default:
      return "UNKNOWN";
  }
}

static const char *iir_filter_to_str(BME69x_IIRFilter filter) {
  switch (filter) {
    case BME69x_IIR_FILTER_OFF:
      return "OFF";
    case BME69x_IIR_FILTER_1X:
      return "1x";
    case BME69x_IIR_FILTER_3X:
      return "3x";
    case BME69x_IIR_FILTER_7X:
      return "7x";
    case BME69x_IIR_FILTER_15X:
      return "15x";
    case BME69x_IIR_FILTER_31X:
      return "31x";
    case BME69x_IIR_FILTER_63X:
      return "63x";
    case BME69x_IIR_FILTER_127X:
      return "127x";
    default:
      return "UNKNOWN";
  }
}

}  // namespace bme69x_base
}  // namespace esphome
