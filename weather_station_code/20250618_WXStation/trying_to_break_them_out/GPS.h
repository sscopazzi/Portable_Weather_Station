#include "functions.h"
extern float humidity, tempC, tempF, pressurehPa;

#ifndef GPS
#define GPS

#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPSLOC(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

// void setupGPS();
// void loopGPS();

#endif