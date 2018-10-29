/**
 @file FaBoColor_BH1749NUC.cpp
 @brief This is a library for the FaBo Color I2C Brick.

   http://fabo.io/230.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include "FaBoColor_BH1749NUC.h"

/**
 @brief Constructor
*/
FaBoColor::FaBoColor(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();
}

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
*/
bool FaBoColor::begin() {
  if ( searchDevice() ) {
    // set default
    setIRGainMode(BH1749NUC_MC1_IR_GAIN_X1);
    setRGBGainMode(BH1749NUC_MC1_RGB_GAIN_X1);
    setMeasurementMode(BH1749NUC_MC1_MEAS_35MS);
    setMeasurementEnable(true);
    return true;
  } else {
    return false;
  }
}

/**
 @brief Search Device
 @retval true device connected
 @retval false device error
*/
bool FaBoColor::searchDevice() {
  uint8_t data;

  readI2c(BH1749NUC_REG_SYSTEM_CONTROL, 1, &data);
//   Serial.println(data,HEX);
//   Serial.println(data & BH1749NUC_SC_PART_ID_MASK,HEX);

  if ( (data & BH1749NUC_SC_PART_ID_MASK) != BH1749NUC_SC_PART_ID ) {
    return false;
  }

  readI2c(BH1749NUC_REG_MANUFACTURER_ID, 1, &data);
//   Serial.println(data,HEX);

  if ( data != BH1749NUC_MANUFACTURER_ID ) {
    return false;
  }

  return true;
}

/**
 @brief Read IR Gain Mode
 @param [out] gain IR Gain mode
*/
uint8_t FaBoColor::readIRGainMode() {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= BH1749NUC_MC1_IR_GAIN_MASK;
  data = data >> BH1749NUC_MC1_IR_GAIN_SHIFT;

  return data;
}

/**
 @brief Set IR Gain Mode
 @param [in] gain IR Gain mode
*/
bool FaBoColor::setIRGainMode(uint8_t mode) {
  uint8_t data;

  if ((mode != BH1749NUC_MC1_IR_GAIN_X1) &&
      (mode != BH1749NUC_MC1_IR_GAIN_X32)) {
    return false;
  }

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= ~(BH1749NUC_MC1_IR_GAIN_MASK);
  data |= (mode << BH1749NUC_MC1_IR_GAIN_SHIFT);
  writeI2c(BH1749NUC_REG_MODE_CONTROL1, data);

  return true;
}

/**
 @brief Read RGB Gain Mode
 @param [out] gain RGB Gain mode
*/
uint8_t FaBoColor::readRGBGainMode() {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= BH1749NUC_MC1_RGB_GAIN_MASK;
  data = data >> BH1749NUC_MC1_RGB_GAIN_SHIFT;

  return data;
}

/**
 @brief Set RGB Gain Mode
 @param [in] gain RGB Gain mode
*/
bool FaBoColor::setRGBGainMode(uint8_t mode) {
  uint8_t data;

  if ((mode != BH1749NUC_MC1_RGB_GAIN_X1) &&
      (mode != BH1749NUC_MC1_RGB_GAIN_X32)) {
    return false;
  }

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= ~(BH1749NUC_MC1_RGB_GAIN_MASK);
  data |= (mode << BH1749NUC_MC1_RGB_GAIN_SHIFT);
  writeI2c(BH1749NUC_REG_MODE_CONTROL1, data);

  return true;
}

/**
 @brief Read Measurement Mode
 @param [out] mode Measurement mode
*/
uint8_t FaBoColor::readMeasurementMode() {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= BH1749NUC_MC1_MEAS_MASK;

  return data;
}

/**
 @brief Set Measurement Mode
 @param [in] mode Measurement mode
*/
bool FaBoColor::setMeasurementMode(uint8_t mode) {
  uint8_t data;

  if ((mode != BH1749NUC_MC1_MEAS_120MS) &&
      (mode != BH1749NUC_MC1_MEAS_240MS) &&
      (mode != BH1749NUC_MC1_MEAS_35MS)) {
    return false;
  }

  readI2c(BH1749NUC_REG_MODE_CONTROL1, 1, &data);
  data &= ~(BH1749NUC_MC1_MEAS_MASK);
  data |= mode;
  writeI2c(BH1749NUC_REG_MODE_CONTROL1, data);

  return true;
}

/**
 @brief Read Measurement Enable
 @param [out] mode Measurement enable
*/
bool FaBoColor::readMeasurementEnable() {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL2, 1, &data);
  data &= BH1749NUC_MC2_RGB_EN_MASK;
  data = data >> BH1749NUC_MC2_RGB_EN_SHIFT;

  return data;
}

/**
 @brief Set Measurement Enable
 @param [in] mode Measurement enable
*/
void FaBoColor::setMeasurementEnable(bool enable) {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL2, 1, &data);
  data &= ~(BH1749NUC_MC2_RGB_EN_MASK);
  data |= ((uint8_t)enable << BH1749NUC_MC2_RGB_EN_SHIFT);
  writeI2c(BH1749NUC_REG_MODE_CONTROL2, data);

}

/**
 @brief Read VALID Register
 @param [out] valid register
*/
bool FaBoColor::readValid() {
  uint8_t data;

  readI2c(BH1749NUC_REG_MODE_CONTROL2, 1, &data);
  data &= BH1749NUC_MC2_VALID_MASK;
  data = data >> BH1749NUC_MC2_VALID_SHIFT;

  return data;
}

/**
 @brief Read Color Data
 @param [out] color data
*/
void FaBoColor::readColor(rgb_data * rgb) {
  uint8_t data[12];

  readI2c(BH1749NUC_REG_RED_DATA, 12, data);

  rgb->red    = data[0] | ( data[1] << 8 );
  rgb->green  = data[3] | ( data[2] << 8 );
  rgb->blue   = data[5] | ( data[4] << 8 );
  rgb->ir     = data[9] | ( data[8] << 8 );
  rgb->green2 = data[11] | ( data[10] << 8 );

}

/**
 @brief Check Color Data
 @param [out] color data
*/
bool FaBoColor::checkData() {
  if( readValid() ) {
    readColor(&this->rgbData);
    return true;
  }
  return false;
}

/**
 @brief Read Red Data
 @param [out] red data
*/
uint16_t FaBoColor::readRed() {
  uint8_t data[2];
  readI2c(BH1749NUC_REG_RED_DATA, 2, data);
  return data[0] | ( data[1] << 8 );
}

/**
 @brief Read Green Data
 @param [out] green data
*/
uint16_t FaBoColor::readGreen() {
  uint8_t data[2];
  readI2c(BH1749NUC_REG_GREEN_DATA, 2, data);
  return data[0] | ( data[1] << 8 );
}

/**
 @brief Read Blue Data
 @param [out] blue data
*/
uint16_t FaBoColor::readBlue() {
  uint8_t data[2];
  readI2c(BH1749NUC_REG_BLUE_DATA, 2, data);
  return data[0] | ( data[1] << 8 );
}

/**
 @brief Read IR Data
 @param [out] IR data
*/
uint16_t FaBoColor::readIR() {
  uint8_t data[2];
  readI2c(BH1749NUC_REG_IR_DATA, 2, data);
  return data[0] | ( data[1] << 8 );
}

/**
 @brief Read Green2 Data
 @param [out] green2 data
*/
uint16_t FaBoColor::readGreen2() {
  uint8_t data[2];
  readI2c(BH1749NUC_REG_GREEN2_DATA, 2, data);
  return data[0] | ( data[1] << 8 );
}


/**
 @brief Write I2C
 @param [in] address register address
 @param [in] data write data
*/
void FaBoColor::writeI2c(uint8_t address, uint8_t data) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

/**
 @brief Read I2C
 @param [in] address register address
 @param [in] num read length
 @param [out] data read data
*/
void FaBoColor::readI2c(uint8_t address, uint8_t num, uint8_t * data) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(address);
  Wire.endTransmission();
  uint8_t i = 0;
  Wire.requestFrom(_i2caddr, num);
  while( Wire.available() ) {
    data[i++] = Wire.read();
  }
}
