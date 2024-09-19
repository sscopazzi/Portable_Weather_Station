// Declare external variables
extern float humidity;
extern float tempC;
extern float tempF;
extern float pressurehPa;

extern int gpsHour, gpsMinute, gpsSecond, gpsMilliseconds;
extern int gpsDay, gpsMonth, gpsYear;
extern bool gpsFix;
extern int gpsFixQuality;
extern float gpsLatitude, gpsLongitude, gpsSpeed, gpsAngle, gpsAltitude;
extern int gpsSatellites;
extern char gpsLatDir, gpsLonDir;
extern float latDecimalDegrees, lonDecimalDegrees; 

/*********************************************************************
  This is an example for our Monochrome OLEDs based on SH1107 drivers

  This example is for a 128x128 size display using I2C to communicate

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);


// for display test
void cat(){
  display.println("       |\\___/|");
  display.println("       )     (");
  display.println("      =\\     /=");
  display.println("        )===( ");
  display.println("       /     \\");
  display.println("       |     |");
  display.println("      /       \\");
  display.println("      \\       /");
  display.println("       \\_____/");
}

// cuz it is nice
void heart(){
  display.println(F("      ,d88b.d88b,"));
  display.println(F("      88888888888"));
  display.println(F("      `Y8888888Y'"));
  display.println(F("        `Y888Y'"));
  display.println(F("          `Y'"));
}

void setupDisplay(){
  if (!display.begin(0x3D)){
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }

  display.clearDisplay();             // Clear the buffer, if it exists 
  // display.setRotation(2);             // it's mounted upside down rn
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  cat();
  // display.println("");
  // NOTE: You _must_ call display stbder making any drawing commands to make them visible on the display hardware!
  // display.display(); // Show the display buffer on the hardware.
}

void loopDisplay(){
  display.clearDisplay();             // Clear the buffer, if it exists 
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  // send stbd lat/lon to OLED
  display.print("C: "); display.println(tempC);
  display.print("F: "); display.println(tempF);
  display.print("H: "); display.println(humidity); 
  display.print("P: "); display.println(pressurehPa); 
  display.print("La: "); display.println(latDecimalDegrees,3); 
  display.print("Lo:"); display.println(lonDecimalDegrees,2);
  display.print("Sp: "); display.println(gpsSpeed); 
  display.print("Al: "); display.print(gpsAltitude);


  display.display();
  // loopnum = loopnum + 1;
}
