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

// BUTTON THINGS
extern const int buttonPin; 
extern int displayMode;
extern int numDisplays;  
extern bool buttonPressed;
extern unsigned long lastDebounceTime;
extern unsigned long debounceDelay; 

// SHOW HISTORY THINGS
#include <SdFat.h>
extern const char filename[];
extern SdFat SD;
extern File32 dataFile;  // Declare the file object as external

extern float vBat;

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
  delay(200);
  if (!display.begin(0x3D)){
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }

  display.clearDisplay();             // Clear the buffer, if it exists 
  display.setRotation(3);             // it's mounted upside down rn
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  
  cat();
  display.println("");
  display.println("");
  display.println("Starting system...");
  display.println();
  display.display();
  delay(4000);
  // NOTE: You _must_ call display stbder making any drawing commands to make them visible on the display hardware!
  // display.display(); // Show the display buffer on the hardware.
}

void loopDisplayAll(){
  display.clearDisplay();             // Clear the buffer, if it exists 
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  // send stbd lat/lon to OLED
  display.println("Time: ");
  display.print((gpsHour < 10 ? "0" : "")); display.print(gpsHour); display.print(':');
  display.print((gpsMinute < 10 ? "0" : "")); display.print(gpsMinute); display.print(':');
  display.print((gpsSecond < 10 ? "0" : "")); display.println(gpsSecond); 

  display.print("Volt: "); display.println(vBat);
  display.print("Cel : "); display.println(tempC);
  display.print("Far : "); display.println(tempF);
  display.print("Hum : "); display.println(humidity); 
  display.print("Pres: "); display.println(pressurehPa); 
  display.print("Lat : "); display.println(latDecimalDegrees,3); 
  display.print("Lon : "); display.println(lonDecimalDegrees,2);
  display.print("Spe : "); display.println(gpsSpeed); 
  display.print("Alt : "); display.println(gpsAltitude);
  display.print("Sat : "); display.println(gpsSatellites);

  display.display();
}

void loopDisplayLoc(){
  display.clearDisplay();             // Clear the buffer, if it exists 
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.print("latDeg: "); display.println(latDecimalDegrees,6); 
  display.print("lat   : "); display.println(gpsLatitude,2); 
  display.println("        DDMM.mm");
  display.println("");
  display.print("lonDeg: "); display.println(lonDecimalDegrees,6);
  display.print("lon   : "); display.println(gpsLongitude,2);
  display.println("        DDMM.mm");
  
  display.println("");
  display.print("Sp: ");  display.println(gpsSpeed); 
  display.print("Al: ");  display.println(gpsAltitude);
  display.println("");
  display.print("vBat");  display.println(vBat);

  display.display();
}


void checkButton() {
  int buttonState = digitalRead(buttonPin);

  // Check if button is pressed and debounce
  if (buttonState == LOW && !buttonPressed && (millis() - lastDebounceTime > debounceDelay)) {
    buttonPressed = true;
    lastDebounceTime = millis();
    
    // Increment the display mode and wrap around
    displayMode = (displayMode + 1) % numDisplays;
  }

  // Reset button pressed state
  if (buttonState == HIGH) {
    buttonPressed = false;
  }
}

void getPressureData(float pressureData[], int totalPoints) {
  // dataFile = SD.open(filename, FILE_WRITE);
  dataFile.open(filename, O_WRITE | O_CREAT | O_APPEND);

  const int maxLines = 2160; // 6 hours of data at 10 seconds per entry
  String lines[maxLines];
  int lineCount = 0;

  // Read the file line by line, store available lines
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    if (lineCount < maxLines) {
      lines[lineCount] = line;
      lineCount++;
    } else {
      // Shift if exceeded maxLines (not likely if filling in data)
      for (int i = 1; i < maxLines; i++) {
        lines[i-1] = lines[i];
      }
      lines[maxLines-1] = line;
    }
  }
  dataFile.close();

  // If we have fewer than 2160 lines, adjust accordingly
  int availableLines = lineCount;
  if (availableLines < totalPoints) {
    totalPoints = availableLines;  // Adjust total points to what we have
  }

  int stepSize = maxLines / totalPoints;

  for (int i = 0; i < totalPoints; i++) {
    String line = lines[i * stepSize];
    int commaIndex = 0;
    int column = 0;

    // Find the 5th column (pressure) by counting commas
    for (int j = 0; j < line.length(); j++) {
      if (line.charAt(j) == ',') {
        commaIndex++;
        if (commaIndex == 4) {
          int nextComma = line.indexOf(',', j + 1);
          String pressureStr = line.substring(j + 1, nextComma);
          pressureData[i] = pressureStr.toFloat();
          break;
        }
      }
    }
  }

  // If fewer than 128 points, pad the remaining with NaN or zeros
  for (int i = totalPoints; i < 128; i++) {
    pressureData[i] = NAN; // Or use 0 if preferred
  }
}

void drawPressureGraph(int graphTop, int graphWidth, int graphHeight) {
    int numPoints = 360; // Example number of points (6 hours of data at 60s intervals)
    float pressureData[numPoints];  // Array of pressure data points

    // Assume getPressureData is already implemented to fill pressureData array
    getPressureData(pressureData, numPoints);

    // Define min and max pressure for y-axis limits
    float minPressure = 950.0;  // Minimum y-limit
    float maxPressure = 1030.0; // Maximum y-limit

    // Draw the graph line
    for (int i = 0; i < numPoints - 1; i++) {
        // Calculate x positions across the graph width
        int x1 = (i * graphWidth) / numPoints;
        int x2 = ((i + 1) * graphWidth) / numPoints;

        // Scale pressure to fit within the graphHeight based on the new limits
        int y1 = graphTop + graphHeight - ((pressureData[i] - minPressure) * graphHeight / (maxPressure - minPressure));
        int y2 = graphTop + graphHeight - ((pressureData[i + 1] - minPressure) * graphHeight / (maxPressure - minPressure));

        // Draw line between points
        display.drawLine(x1, y1, x2, y2, SH110X_WHITE);
    }
}

void loopDisplayPressureHistory() {
    display.clearDisplay();

    // Display the pressure at the top
    display.setCursor(0, 0);
    display.setTextSize(1);          // Normal 1:1 pixel scale
    display.setTextColor(SH110X_WHITE);
    display.print("P: "); display.println(pressurehPa);    // Pressure display, taking up space at the top

    // Reserve 16 pixels for the text at the top and use the remaining 112 pixels for the graph
    const int graphHeight = 112;     // Available height for the graph
    const int graphWidth = 128;      // Full width for the graph
    const int graphTop = 16;         // Start plotting the graph below the text (16px reserved for text)

    // Draw the graph in the remaining space
    drawPressureGraph(graphTop, graphWidth, graphHeight);

    // Send everything to the display
    display.display();
}

// void waitForGPSTime() {
//   display.clearDisplay();             
//   display.setTextSize(1);             
//   display.setTextColor(SH110X_WHITE); 
//   display.setCursor(0, 0);

//   display.println("  WAITING FOR GPS FIX");
//   display.println("");
//   display.println("  Searching for satellites...");
//   display.println("");
//   display.print("  Sats: "); display.println(GPS.satellites);
//   display.print("  Fix:  "); display.println(GPS.fix ? "YES" : "NO");
//   display.println("");
//   display.println("  Move antenna outside");
//   display.display();

//   Serial.print("Satellites: ");
//   Serial.print(GPS.satellites);
//   Serial.print("  Fix: ");
//   Serial.println(GPS.fix ? "YES" : "NO");
// }







// // Initialize the SD card
// void setupSD() {
// // while (!Serial) { yield(); delay(10); }     // wait till serial port is opened
//   delay(100);  // RP2040 delay is not a bad idea

//   Serial.print("Initializing SD card...");

//   // Retry mechanism for SD card initialization
//   while (!SD.begin(config)) {
//     Serial.println("Card failed, or not present :(");
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SH110X_WHITE);
//     display.setCursor(0, 0);
//     display.println("    SD card FAILED");
//     display.println("");
//     display.println("  RESTART or PLUG IN");
//     display.println("");
//     display.println("  <xxx> <xxx> <xxx>");
//     display.println("");
//     display.println("If it still doesn't");
//     display.println("work you may need to");
//     display.println("REFORMAT the card");
//     display.println("");
//     display.println("Try exFAT");
//     display.display();
//     while (1) {}  // Halt the system until the SD card is fixed
//   }

//   // Get filename from GPS
//   timestamp_filename = getGPSFilename();
//   Serial.print("Filename: ");
//   Serial.println(timestamp_filename);

//   // Open file and write the header
//   File32 myFile = SD.open(timestamp_filename + ".csv", FILE_WRITE);
//   if (myFile) {
//     myFile.println("time,tempC,tempF,humidity,pressurehPa,fix,fixQual,lat,latDir,lon,lonDir,speed,angle,alt,satNum,vBat");
//     myFile.close();
//     Serial.println("File created and header written!");
//   } else {
//     Serial.println("Error creating file!");
//   }

//   // // Open file and write the header once
//   // dataFile.open(filename, O_WRITE | O_CREAT | O_APPEND);
//   // if (dataFile) {
//   //   dataFile.println("time,tempC,tempF,humidity,pressurehPa,fix,fixQual,lat,latDir,lon,lonDir,speed,angle,alt,satNum,vBat");
//   //   dataFile.close();
//   //   Serial.println("Card initialized! Header added.");
//   // }
// }
