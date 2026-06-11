// #include "sdCard.h"
// #include "display.h"

// SdFat SD;  // Declare and initialize the SD object
// File dataFile;  // Declare the data file object

// const char filename[] = "20240915_weatherstation0";


// void setupSD() {
//     Serial.println("Initializing SD card...");
//     if (!SD.begin(config)) {
//         Serial.println("Card failed, or not present :(");
//         display.clearDisplay();
//         display.setTextSize(1);
//         display.setTextColor(SH110X_WHITE);
//         display.setCursor(0, 0);
//         display.println("    SD card FAILED");
//         display.println("");
//         display.println("  RESTART or PLUG IN");
//         display.println("");
//         display.println("  <xxx> <xxx> <xxx>");
//         display.println("");
//         display.println("If it still doesn't work you may need to REFORMAT the card");
//         display.println("");
//         display.println("use exFAT");
//         display.display();
//         while (1) {}
//     }

//     dataFile = SD.open(filename, FILE_WRITE);
//     if (dataFile) {
//         dataFile.println("tempC,tempF,humidity,pressurehPa");
//         dataFile.close();
//         Serial.println("Card initialized!");
//     }
// }

// void loopSD() {
//     dataFile = SD.open(filename, FILE_WRITE);
//     if (dataFile) {
//         dataFile.print(tempC);  dataFile.print(",");
//         dataFile.print(tempF);  dataFile.print(",");
//         dataFile.print(humidity);  dataFile.print(",");
//         dataFile.println(pressurehPa);
//         dataFile.close();
//     } else {
//         Serial.println("error opening file on SD card...");
//         display.clearDisplay();
//         display.setTextSize(1);
//         display.setTextColor(SH110X_WHITE);
//         display.setCursor(0, 0);
//         display.println("    SD card FAILED");
//         display.println("");
//         display.println("  RESTART or PLUG IN");
//         display.println("");
//         display.println("  <xxx> <xxx> <xxx>");
//         display.println("");
//         display.println("If it still doesn't work you may need to REFORMAT the card");
//         display.println("");
//         display.println("use exFAT");
//         display.display();
//         while (1) {}
//     }
// }


#include "sdCard.h"
#include "display.h"

// Initialize the SD object and file object in the .cpp file
SdFat SD;  
File32 dataFile;  // Use File32 for SdFat

// Filename for the data file on SD card
const char filename[] = "20240915_weatherstation0";

// SPI configuration for SD card
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

void setupSD() {
    Serial.println("Initializing SD card...");
    if (!SD.begin(config)) {
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
        display.println("If it still doesn't work you may need to REFORMAT the card");
        display.println("");
        display.println("use exFAT");
        display.display();
        while (1) {}  // Halt the system until the SD card is fixed
    }

    // Open file and write the header once
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.println("tempC,tempF,humidity,pressurehPa");
        dataFile.close();
        Serial.println("Card initialized!");
    }
}

void loopSD() {
    // Open the file for appending data
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.print(tempC);  
        dataFile.print(",");
        dataFile.print(tempF);  
        dataFile.print(",");
        dataFile.print(humidity);  
        dataFile.print(",");
        dataFile.println(pressurehPa);
        dataFile.close();  // Close after writing
    } else {
        Serial.println("error opening file on SD card...");
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
        display.println("If it still doesn't work you may need to REFORMAT the card");
        display.println("");
        display.println("use exFAT");
        display.display();
        while (1) {}  // Halt system on SD failure
    }
}
