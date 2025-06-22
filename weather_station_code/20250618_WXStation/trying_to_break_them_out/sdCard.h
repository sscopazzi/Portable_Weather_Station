// #include "functions.h"
// extern float humidity, tempC, tempF, pressurehPa;

// extern const char filename[];

// #include "SdFat.h"

// // extern SdFat SD;

// // extern File32 dataFile;

// #define SD_CS_PIN 23
// SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);


#ifndef SD_CARD_H
#define SD_CARD_H

#include "functions.h"
extern float humidity, tempC, tempF, pressurehPa;

extern const char filename[];

#include "SdFat.h"

// Only declare the SD object, no need to initialize here
extern SdFat SD;
extern File32 dataFile;  // Use File32 for SdFat

#define SD_CS_PIN 23
extern SdSpiConfig config;

// void setupSD();
// void loopSD();

#endif // SD_CARD_H
