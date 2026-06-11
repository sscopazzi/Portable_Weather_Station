#include "display.h"

Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);  // Define the display object


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

void heart(){
  display.println(F("  ,d88b.d88b,"));
  display.println(F("  88888888888"));
  display.println(F("  `Y8888888Y'"));
  display.println(F("    `Y888Y'"));
  display.println(F("      `Y'"));
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
  display.println("Starting system...");
  display.println("");
  display.println("");
  cat();

  // NOTE: You _must_ call display stbder making any drawing commands to make them visible on the display hardware!
  display.display(); // Show the display buffer on the hardware.
  delay(1000);
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
  display.print("P: "); display.print(pressurehPa); // space between

  display.display();
  // loopnum = loopnum + 1;
}