/**
 @file readColor.ino
 @brief This is an Example for the FaBo Color I2C Brick.

   http://fabo.io/230.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include <Wire.h>
#include <FaBoColor_BH1749NUC.h>

FaBoColor FaBoColor(BH1749NUC_SLAVE_ADDRESS_DEFAULT);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("RESET");
  Serial.println();
  Serial.println("configuring device.");

  if (FaBoColor.begin()) {
    Serial.println("configured FaBo Color Brick");
  } else {
    Serial.println("device error");
    while(1);
  }

  FaBoColor.setIRGainMode(BH1749NUC_MC1_IR_GAIN_X1);
  FaBoColor.setRGBGainMode(BH1749NUC_MC1_RGB_GAIN_X1);
  FaBoColor.setMeasurementMode(BH1749NUC_MC1_MEAS_240MS);

}

void loop() {
  if (FaBoColor.checkData()) {
    Serial.print("Red: ");
    Serial.println(FaBoColor.rgbData.red);
    Serial.print("Green: ");
    Serial.println(FaBoColor.rgbData.green);
    Serial.print("Blue: ");
    Serial.println(FaBoColor.rgbData.blue);
    Serial.print("IR: ");
    Serial.println(FaBoColor.rgbData.ir);
    Serial.print("Green2: ");
    Serial.println(FaBoColor.rgbData.green2);
  }

}
