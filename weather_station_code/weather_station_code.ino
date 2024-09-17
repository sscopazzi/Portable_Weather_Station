#include "display.h"

float humidity;
float tempC;
float tempF;
float pressurehPa;

const char filename[] = "20240917_weatherstation_walkingtest.txt";

int gpsHour, gpsMinute, gpsSecond, gpsMilliseconds;
int gpsDay, gpsMonth, gpsYear;
bool gpsFix;
int gpsFixQuality;
float gpsLatitude, gpsLongitude, gpsSpeed, gpsAngle, gpsAltitude;
int gpsSatellites;
char gpsLatDir, gpsLonDir;

#include <SPI.h>
#include "SdFat.h"

#define SD_CS_PIN 23

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);


// Initialize the SD card
void setupSD() {
while (!Serial) { yield(); delay(10); }     // wait till serial port is opened
  delay(100);  // RP2040 delay is not a bad idea

  Serial.print("Initializing SD card...");

  // Retry mechanism for SD card initialization
  while (!SD.begin(config)) {
    Serial.println("Card failed, or not present :(");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("    SD card FAILED");
    display.println("");
    display.println("  RESTART or PLUG IN");
    display.println("");
    display.println("  <xxx> <xxx> <xxx>");
    display.println("");
    display.println("If it still doesn't");
    display.println("work you may need to");
    display.println("REFORMAT the card");
    display.println("");
    display.println("Try exFAT");
    display.display();
    while (1) {}  // Halt the system until the SD card is fixed
  }

  // Open file and write the header once
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,tempC,tempF,humidity,pressurehPa,fix,fixQual,lat,latDir,lon,lonDir,speed,angle,alt,satNum");
    dataFile.close();
    Serial.println("Card initialized! Header added.");
  }
}

// Append data to the SD card
void loopSD() {
  // Open the file for appending data
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.print(gpsYear);        dataFile.print("-");
    dataFile.print(gpsMonth);       dataFile.print("-");
    dataFile.print(gpsDay);         dataFile.print(" ");
    dataFile.print(gpsHour);        dataFile.print(":");
    dataFile.print(gpsMinute);      dataFile.print(":"); // in one column as Pandas (Python library) expects
    dataFile.print(gpsSecond);      dataFile.print(",");
    dataFile.print(tempC);          dataFile.print(",");
    dataFile.print(tempF);          dataFile.print(",");
    dataFile.print(humidity);       dataFile.print(",");
    dataFile.print(pressurehPa);    dataFile.print(",");
    dataFile.print(gpsFix);         dataFile.print(",");
    dataFile.print(gpsFixQuality);  dataFile.print(",");
    dataFile.print(gpsLatitude,4);  dataFile.print(",");
    dataFile.print(gpsLatDir);      dataFile.print(",");
    dataFile.print(gpsLongitude,4); dataFile.print(",");
    dataFile.print(gpsLonDir);      dataFile.print(",");
    dataFile.print(gpsSpeed);       dataFile.print(",");
    dataFile.print(gpsAngle);       dataFile.print(",");
    dataFile.print(gpsAltitude);    dataFile.print(",");
    dataFile.println(gpsSatellites);
    dataFile.close();  // Close after writing

  } else {
    Serial.println("Card failed, or not present :(");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("    SD card FAILED");
    display.println("");
    display.println("  RESTART or PLUG IN");
    display.println("");
    display.println("  <xxx> <xxx> <xxx>");
    display.println("");
    display.println("If it still doesn't");
    display.println("work you may need to");
    display.println("REFORMAT the card");
    display.println("");
    display.println("Try exFAT");
    display.display();
    while (1) {}  // Halt the system until the SD card is fixed
  }
}


// TEMP AND HUMIDITY SHT45
/*************************************************** 
  This is an example for the SHT4x Humidity & Temp Sensor

  Designed specifically to work with the SHT4x sensor from Adafruit
  ----> https://www.adafruit.com/products/4885

  These sensors use I2C to communicate, 2 pins are required to  
  interface
 ****************************************************/

#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

void setupSHT45(){
  Serial.println("Adafruit SHT4x test");
    if (! sht4.begin()) {
      Serial.println("Couldn't find SHT4x");
      while (1) delay(1);
    }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  
    switch (sht4.getPrecision()) {
        case SHT4X_HIGH_PRECISION: 
          Serial.println("High precision");
          break;
        case SHT4X_MED_PRECISION: 
          Serial.println("Med precision");
          break;
        case SHT4X_LOW_PRECISION: 
          Serial.println("Low precision");
          break;
    }

  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_HIGH_HEATER_1S);
    switch (sht4.getHeater()) {
        case SHT4X_NO_HEATER: 
          Serial.println("No heater");
          break;
        case SHT4X_HIGH_HEATER_1S: 
          Serial.println("High heat for 1 second");
          break;
        case SHT4X_HIGH_HEATER_100MS: 
          Serial.println("High heat for 0.1 second");
          break;
        case SHT4X_MED_HEATER_1S: 
          Serial.println("Medium heat for 1 second");
          break;
        case SHT4X_MED_HEATER_100MS: 
          Serial.println("Medium heat for 0.1 second");
          break;
        case SHT4X_LOW_HEATER_1S: 
          Serial.println("Low heat for 1 second");
          break;
        case SHT4X_LOW_HEATER_100MS: 
          Serial.println("Low heat for 0.1 second");
          break;
    }
}

void loopSHT45(){
  // Rename local variables to avoid name conflict
  sensors_event_t humidityEvent, tempEvent;
  
  uint32_t timestamp = millis();  // Record time before reading
  sht4.getEvent(&humidityEvent, &tempEvent);  // Populate temp and humidityEvent with fresh data
  timestamp = millis() - timestamp;  // Calculate how long the reading took

  // Print sensor readings to Serial
  // Serial.print("Temperature: "); 
  // Serial.print(tempEvent.temperature); 
  // Serial.println(" degrees C");

  Serial.print("Humidity: "); 
  Serial.print(humidityEvent.relative_humidity); 
  Serial.println("% rH");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);

  // Assign the values to your global variables
  // tempC = tempEvent.temperature;
  // tempF = (tempC * 9 / 5) + 32;
  humidity = humidityEvent.relative_humidity;  // Use the correct global humidity variable
}



// LOCATION PARSING
// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to parse data from the I2C GPS
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


void setupLOC(){
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  // Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");
  delay(250);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 16 62.5 seconds, 100 10 seconds 200 5 seconds

  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  // delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loopLOC(){
    // read data from the GPS in the 'main loop'
    // char c = GPS.read();
    // // if you want to debug, this is a good time to do it!
    // // Serial.println(c);
    // // if (GPSECHO)
    // //   if (c) Serial.print(c);
    // // if a sentence is received, we can check the checksum, parse it...
    // if (GPS.newNMEAreceived()) {
    //   // a tricky thing here is if we print the NMEA sentence, or data
    //   // we end up not listening and catching other sentences!
    //   // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //   // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    //   if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    //     return; // we can fail to parse a sentence in which case we should just wait for another
    // }

  // Wait for 10 seconds using a blocking loop
  unsigned long waitStart = millis(); // Capture the current time
  while (millis() - waitStart < 10000) {
    // This loop will hold the program for 10 seconds
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  // Once 10 seconds have passed, process GPS data
  // timer = millis(); // Reset the timer after the wait

    // if (millis() - timer > 2000) {
    //   timer = millis(); // reset the timer
      // Assign GPS data to global variables
      gpsHour = GPS.hour;
      gpsMinute = GPS.minute;
      gpsSecond = GPS.seconds;
      gpsMilliseconds = GPS.milliseconds;
      
      gpsDay = GPS.day;
      gpsMonth = GPS.month;
      gpsYear = GPS.year;

      gpsFix = GPS.fix;
      gpsFixQuality = GPS.fixquality;

      if (gpsFix) {
        gpsLatitude = (GPS.latitude)/100;
        gpsLongitude = (GPS.longitude)/100;
        gpsLatDir = GPS.lat;
        gpsLonDir = GPS.lon;
        gpsSpeed = GPS.speed;
        gpsAngle = GPS.angle;
        gpsAltitude = GPS.altitude;
        gpsSatellites = GPS.satellites;
      }

      // Print the values from global variables
      Serial.print("\nTime: ");
      if (gpsHour < 10) { Serial.print('0'); }
      Serial.print(gpsHour); Serial.print(':');
      if (gpsMinute < 10) { Serial.print('0'); }
      Serial.print(gpsMinute); Serial.print(':');
      if (gpsSecond < 10) { Serial.print('0'); }
      Serial.println(gpsSecond);// Serial.print('.');
      // if (gpsMilliseconds < 10) {
      //   Serial.print("00");
      // } else if (gpsMilliseconds > 9 && gpsMilliseconds < 100) {
      //   Serial.print("0");
      // }
      // Serial.println(gpsMilliseconds);

      Serial.print("Date: ");
      Serial.print(gpsMonth); Serial.print('/');
      Serial.print(gpsDay); Serial.print('/');
      Serial.println(gpsYear);

      Serial.print("Fix: "); Serial.print(gpsFix);
      Serial.print(" quality: "); Serial.println(gpsFixQuality);

      if (gpsFix) {
        Serial.print("Location: ");
        Serial.print(gpsLatitude, 4); Serial.print(gpsLatDir);
        Serial.print(", ");
        Serial.print(gpsLongitude, 4); Serial.println(gpsLonDir);
        Serial.print("Speed (knots): "); Serial.println(gpsSpeed);
        Serial.print("Angle: "); Serial.println(gpsAngle);
        Serial.print("Altitude: "); Serial.println(gpsAltitude);
        Serial.print("Satellites: "); Serial.println(gpsSatellites);
      }
    // }
}


// BMP390
/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25) // used for altitude but fuck that cuz it bad

Adafruit_BMP3XX bmp;

void setupBMP390(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loopBMP390(){
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  pressurehPa = bmp.pressure/100.0;
  tempC = bmp.temperature;
  tempF = (tempC * 9 / 5) + 32;

  Serial.println();
}



void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(200);     // will pause Zero, Leonardo, etc until serial console opens

  // UTILITY  
  setupDisplay();
  display.println("");
  display.println("Starting system...");
  display.println("");
  display.display();
  delay(1000);
  setupSD();
  display.println("SD Card  healthy!");
  display.display();

  // SENSORS
  setupLOC();
  display.println("GPS      healthy!");
  display.display();
  delay(300);

  setupSHT45();
  display.println("SHT45    healthy!");
  display.display();
  delay(300);

  setupBMP390();
  display.println("BMP390   healthy!");
  display.println("");
  display.display();
  delay(5000);
  
  display.clearDisplay();             // Clear the buffer, if it exists 
  // display.setRotation(2);             // it's mounted upside down rn
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(" <<ALL SENSORS INIT>>");
  display.println(" <<ALL SENSORS INIT>>");
  display.println(" <<ALL SENSORS INIT>>");
  display.println("");
  display.println("");
  heart();
  display.println("");
  display.println("");
  display.println("  Sophie LV Scopazzi");
  display.println("  Weather Station v1" );
  display.println("     2024-09-17");
  display.display();
  delay(5000);
}


void loop() {
  // SENSORS
  loopLOC();
  loopSHT45();
  loopBMP390();

  // UTILITY
  loopDisplay();
  loopSD();
}