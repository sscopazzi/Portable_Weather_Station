#include "GPS.h"

void setupGPS(){
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSLOC.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPSLOC.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPSLOC.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPSLOC.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSLOC.println(PMTK_Q_RELEASE);
}

void loopGPS(){
  // read data from the GPS in the 'main loop'
  char c = GPSLOC.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPSLOC.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPSLOC.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPSLOC.parse(GPSLOC.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPSLOC.hour < 10) { Serial.print('0'); }
    Serial.print(GPSLOC.hour, DEC); Serial.print(':');
    if (GPSLOC.minute < 10) { Serial.print('0'); }
    Serial.print(GPSLOC.minute, DEC); Serial.print(':');
    if (GPSLOC.seconds < 10) { Serial.print('0'); }
    Serial.print(GPSLOC.seconds, DEC); Serial.print('.');
    if (GPSLOC.milliseconds < 10) {
      Serial.print("00");
    } else if (GPSLOC.milliseconds > 9 && GPSLOC.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPSLOC.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPSLOC.day, DEC); Serial.print('/');
    Serial.print(GPSLOC.month, DEC); Serial.print("/20");
    Serial.println(GPSLOC.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPSLOC.fix);
    Serial.print(" quality: "); Serial.println((int)GPSLOC.fixquality);
    if (GPSLOC.fix) {
      Serial.print("Location: ");
      Serial.print(GPSLOC.latitude, 4); Serial.print(GPSLOC.lat);
      Serial.print(", ");
      Serial.print(GPSLOC.longitude, 4); Serial.println(GPSLOC.lon);
      Serial.print("Speed (knots): "); Serial.println(GPSLOC.speed);
      Serial.print("Angle: "); Serial.println(GPSLOC.angle);
      Serial.print("Altitude: "); Serial.println(GPSLOC.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPSLOC.satellites);
    }
  }
}