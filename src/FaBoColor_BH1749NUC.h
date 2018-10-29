/**
 @file FaBoColor_BH1749NUC.h
 @brief This is a library for the FaBo Color I2C Brick.

   http://fabo.io/230.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#ifndef FABOCOLOR_BH1749NUC_H
#define FABOCOLOR_BH1749NUC_H

#include <Arduino.h>
#include <Wire.h>

/// @name I2C Slave Address
/// @{
#define BH1749NUC_SLAVE_ADDRESS_L 0x38
#define BH1749NUC_SLAVE_ADDRESS_H 0x39
#define BH1749NUC_SLAVE_ADDRESS_DEFAULT 0x38
/// @}

/// @name Register Map Addresses
/// @{
#define BH1749NUC_REG_SYSTEM_CONTROL 0x40
#define BH1749NUC_REG_MODE_CONTROL1  0x41
#define BH1749NUC_REG_MODE_CONTROL2  0x42
#define BH1749NUC_REG_RED_DATA   0x50
#define BH1749NUC_REG_GREEN_DATA 0x52
#define BH1749NUC_REG_BLUE_DATA  0x54
#define BH1749NUC_REG_IR_DATA    0x58
#define BH1749NUC_REG_GREEN2_DATA 0x5A
#define BH1749NUC_REG_INTERRUPT 0x60
#define BH1749NUC_REG_PERSISTENCE 0x61
#define BH1749NUC_REG_TH_HIGH 0x62
#define BH1749NUC_REG_TH_LOW 0x64
#define BH1749NUC_REG_MANUFACTURER_ID 0x92
/// @}

/// @name SYSTEM CONTROL
/// @{
#define BH1749NUC_SC_PART_ID 0x0D
#define BH1749NUC_SC_PART_ID_MASK 0x3F
/// @}

/// @name Mode Control 1
/// @{
#define BH1749NUC_MC1_IR_GAIN_X1 B01
#define BH1749NUC_MC1_IR_GAIN_X32 B11
#define BH1749NUC_MC1_IR_GAIN_MASK 0x60
#define BH1749NUC_MC1_IR_GAIN_SHIFT 5
#define BH1749NUC_MC1_RGB_GAIN_X1 B01
#define BH1749NUC_MC1_RGB_GAIN_X32 B11
#define BH1749NUC_MC1_RGB_GAIN_MASK 0x18
#define BH1749NUC_MC1_RGB_GAIN_SHIFT 3
#define BH1749NUC_MC1_MEAS_120MS B010
#define BH1749NUC_MC1_MEAS_240MS B011
#define BH1749NUC_MC1_MEAS_35MS  B101
#define BH1749NUC_MC1_MEAS_MASK 0x7
/// @}

/// @name Mode Control 1
/// @{
#define BH1749NUC_MC2_VALID_MASK 0x80
#define BH1749NUC_MC2_VALID_SHIFT 7
#define BH1749NUC_MC2_RGB_EN_MASK 0x10
#define BH1749NUC_MC2_RGB_EN_SHIFT 4
/// @}

/// @name Manufacturer ID
/// @{
#define BH1749NUC_MANUFACTURER_ID 0xE0
/// @}

/// @name RGB Data struct
/// @{
struct rgb_data {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t ir;
  uint16_t green2;
};
/// @}


/**
 @class FaBoColor
 @brief FaBo Color I2C Controll class
*/
class FaBoColor {
  public:
    rgb_data rgbData;
    FaBoColor(uint8_t addr = BH1749NUC_SLAVE_ADDRESS_DEFAULT);
    bool begin(void);
    bool searchDevice(void);
    uint8_t readIRGainMode(void);
    bool setIRGainMode(uint8_t mode);
    uint8_t readRGBGainMode(void);
    bool setRGBGainMode(uint8_t mode);
    uint8_t readMeasurementMode(void);
    bool setMeasurementMode(uint8_t mode);
    bool readMeasurementEnable(void);
    void setMeasurementEnable(bool);
    bool readValid(void);
    void readColor(rgb_data * rgb);
    bool checkData(void);
    uint16_t readRed(void);
    uint16_t readGreen(void);
    uint16_t readBlue(void);
    uint16_t readIR(void);
    uint16_t readGreen2(void);
  private:
    uint8_t _i2caddr;
    void writeI2c(uint8_t address, uint8_t data);
    void readI2c(uint8_t address, uint8_t num, uint8_t * data);
};

#endif // FABOCOLOR_BH1749NUC_H
