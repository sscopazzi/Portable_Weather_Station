// #include "functions.h"
// extern float humidity, tempC, tempF, pressurehPa;

// #ifndef DISP
// #define DISP

// #include <SPI.h>              // ALSO SD
// #include <Wire.h>             // ALSO SENSORS
// #include <Adafruit_GFX.h>     // ONLY THIS
// #include <Adafruit_SH110X.h>  // ONLY THIS
// #define SCREEN_WIDTH 128  // OLED display width, in pixels
// #define SCREEN_HEIGHT 128 // OLED display height, in pixels
// #define OLED_RESET -1     // can set an oled reset pin if desired
// Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);

// /*********************************************************************
//   This is an example for our Monochrome OLEDs based on SH1107 drivers

//   This example is for a 128x128 size display using I2C to communicate

//   Adafruit invests time and resources providing this open source code,
//   please support Adafruit and open-source hardware by purchasing
//   products from Adafruit!

//   Written by Limor Fried/Ladyada  for Adafruit Industries.
//   BSD license, check license.txt for more information
//   All text above, and the splash screen must be included in any redistribution
// *********************************************************************/

// void cat();
// void heart();
// void setupDisplay();
// void loopDisplay();

// #endif


#include "functions.h"
extern float humidity, tempC, tempF, pressurehPa;

#ifndef DISP
#define DISP

#include <SPI.h>              // ALSO SD
#include <Wire.h>             // ALSO SENSORS
#include <Adafruit_GFX.h>     // ONLY THIS
#include <Adafruit_SH110X.h>  // ONLY THIS

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired

extern Adafruit_SH1107 display;  // Declare the display object as extern

// void cat();
// void heart();
// void setupDisplay();
// void loopDisplay();

#endif
