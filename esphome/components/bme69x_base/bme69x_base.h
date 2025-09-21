#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bme69x_base {


                                    

/// Enum listing all IIR Filter options.
enum BME69x_IIRFilter {
  BME69x_IIR_FILTER_OFF = 0b000,
  BME69x_IIR_FILTER_1X = 0b001,
  BME69x_IIR_FILTER_3X = 0b010,
  BME69x_IIR_FILTER_7X = 0b011,
  BME69x_IIR_FILTER_15X = 0b100,
  BME69x_IIR_FILTER_31X = 0b101,
  BME69x_IIR_FILTER_63X = 0b110,
  BME69x_IIR_FILTER_127X = 0b111,
};

/// Enum listing all oversampling options.
enum BME69x_Oversampling {
  BME69x_OVERSAMPLING_NONE = 0b000,
  BME69x_OVERSAMPLING_1X = 0b001,
  BME69x_OVERSAMPLING_2X = 0b010,
  BME69x_OVERSAMPLING_4X = 0b011,
  BME69x_OVERSAMPLING_8X = 0b100,
  BME69x_OVERSAMPLING_16X = 0b101,
};

struct BME69x_CalibrationData
{
    /*! Calibration coefficient for the humidity sensor */
    int16_t par_h1;

    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h2;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t par_h3;

    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h4;

    /*! Calibration coefficient for the humidity sensor */
    int16_t par_h5;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t par_h6;

    /*! Calibration coefficient for the gas sensor */
    int8_t par_g1;

    /*! Calibration coefficient for the gas sensor */
    int16_t par_g2;

    /*! Calibration coefficient for the gas sensor */
    int8_t par_g3;

    /*! Calibration coefficient for the temperature sensor */
    uint16_t par_t1;

    /*! Calibration coefficient for the temperature sensor */
    uint16_t par_t2;

    /*! Calibration coefficient for the temperature sensor */
    int8_t par_t3;

    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p5;

    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p6;

    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p7;

    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p8;

    /*! Calibration coefficient for the pressure sensor */
    uint16_t par_p1;

    /*! Calibration coefficient for the pressure sensor */
    uint16_t par_p2;

    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p3;

    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p4;

    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p9;

    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p10;

    int8_t par_p11;

    /*! Variable to store the intermediate temperature coefficient */
    float t_fine;

    /*! Heater resistance range coefficient */
    uint8_t res_heat_range;

    /*! Heater resistance value coefficient */
    int8_t res_heat_val;

    /*! Gas resistance range switching error coefficient */
    int8_t range_sw_err;
};



class BME69x_Component : public PollingComponent, public i2c::I2CDevice {
 public:
  /// Set the temperature oversampling value. Defaults to 16X.
  void set_temperature_oversampling(BME69x_Oversampling temperature_oversampling);
  /// Set the pressure oversampling value. Defaults to 16X.
  void set_pressure_oversampling(BME69x_Oversampling pressure_oversampling);
  /// Set the humidity oversampling value. Defaults to 16X.
  void set_humidity_oversampling(BME69x_Oversampling humidity_oversampling);
  /// Set the IIR Filter value. Defaults to no IIR Filter.
  void set_iir_filter(BME69x_IIRFilter iir_filter);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  void set_gas_resistance_sensor(sensor::Sensor *gas_resistance_sensor) { gas_resistance_sensor_ = gas_resistance_sensor;}

  /** Configure the internal gas sensor heater.
   *
   * This component enables the heater by default with 320 °C for 150 ms
   * if not overridden in the YAML configuration.
   *
   * @param heater_temperature Target heater temperature in °C (200–400 °C recommended).
   * @param heater_duration Heater on-time in ms (1–4032 ms).
   */
  void set_heater(uint16_t heater_temperature, uint16_t heater_duration);


  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

 protected:

  float ambient_temperature_ = 25.0;

  bool init_bme69x_(void);
  bool read_calibration_data_(void);
  bool set_sensor_configuration_(void);
  bool set_heater_configuration_(void);
  bool set_operating_mode_(uint8_t mode);
  uint8_t calculate_heater_resistance_(uint16_t temperature);
  uint8_t calculate_gas_wait_(uint16_t duration);

  uint16_t concat_bytes_(uint8_t msb, uint8_t lsb);
  uint8_t set_bits_(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t val);
  uint8_t set_bits_pos0_(uint8_t reg, uint8_t mask, uint8_t val);


  /// Calculate the temperature in °C using the provided raw ADC value.
  float calc_temperature_(uint32_t raw_temperature);
  /// Calculate the pressure in hPa using the provided raw ADC value.
  float calc_pressure_(uint32_t raw_pressure);
  /// Calculate the relative humidity in % using the provided raw ADC value.
  float calc_humidity_(uint16_t raw_humidity);
  /// Calculate the gas resistance in Ω using the provided raw ADC value.
  float calc_gas_resistance_(uint16_t raw_gas, uint8_t range);
  /// Calculate how long the sensor will take until we can retrieve data.
  uint32_t calc_meas_duration_();

  BME69x_CalibrationData calibration_;
  BME69x_Oversampling temperature_oversampling_{BME69x_OVERSAMPLING_16X};
  BME69x_Oversampling pressure_oversampling_{BME69x_OVERSAMPLING_16X};
  BME69x_Oversampling humidity_oversampling_{BME69x_OVERSAMPLING_16X};
  BME69x_IIRFilter iir_filter_{BME69x_IIR_FILTER_OFF};
  uint16_t heater_temperature_{320};
  uint16_t heater_duration_{150};

  virtual bool read_byte(uint8_t a_register, uint8_t *data) = 0;
  virtual bool write_byte(uint8_t a_register, uint8_t data) = 0;
  virtual bool write_bytes(uint8_t a_register, const uint8_t *data, uint8_t len) = 0;
  virtual bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) = 0;
  virtual bool read_byte_16(uint8_t a_register, uint16_t *data) = 0;

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *gas_resistance_sensor_{nullptr};
};

}  // namespace bme69x_base
}  // namespace esphome
