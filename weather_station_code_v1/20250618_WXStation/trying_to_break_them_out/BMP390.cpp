// #include "BMP390.h"

// Adafruit_BMP3XX bmp;

// void setupBMP390() {
//   if (!bmp.begin_I2C()) {
//     Serial.println("Could not find a valid BMP3 sensor, check wiring!");
//     while (1);
//   }
//   bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//   bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
//   bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//   bmp.setOutputDataRate(BMP3_ODR_50_HZ);
// }

// void loopBMP390() {
//   if (!bmp.performReading()) {
//     Serial.println("Failed to perform reading :(");
//     return;
//   }
//   Serial.print("Temperature = ");
//   Serial.print(bmp.temperature);
//   Serial.println(" *C");
  
//   Serial.print("Pressure = ");
//   Serial.print(bmp.pressure / 100.0);
//   Serial.println(" hPa");
  
//   pressurehPa = bmp.pressure / 100.0;
//   tempC = bmp.temperature;
//   tempF = (tempC * 9 / 5) + 32;
  
//   Serial.println();
// }

#include "BMP390.h"

Adafruit_BMP3XX bmp;  // Define the bmp object

void setupBMP390() {
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loopBMP390() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  
  pressurehPa = bmp.pressure / 100.0;
  tempC = bmp.temperature;
  tempF = (tempC * 9 / 5) + 32;
  
  Serial.println();
}
