// #include "functions.h"
// extern float humidity, tempC, tempF, pressurehPa;

// #ifndef BMP390_H
// #define BMP390_H

// #include <Adafruit_BMP3XX.h>

// extern Adafruit_BMP3XX bmp;

// // void setupBMP390();
// // void loopBMP390();

// #endif

#ifndef BMP390_H
#define BMP390_H

#include "functions.h"
extern float humidity, tempC, tempF, pressurehPa;

#include <Adafruit_BMP3XX.h>

extern Adafruit_BMP3XX bmp;  // Declare the bmp object as extern

void setupBMP390();
void loopBMP390();

#endif
