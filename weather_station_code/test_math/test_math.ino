// --- GPS Coordinate Conversion Test ---
// Verifies DDMM.mmmm → Decimal Degrees conversion

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- GPS Conversion Test ---");

  // Example test coordinates (from your data)
  testConversion(4134.0923, 'N');
  testConversion(7039.6284, 'W');
  testConversion(4133.6650, 'N');
  testConversion(07039.6220, 'W');

  // Optional: test with known edge cases
  testConversion(9000.0000, 'N'); // North Pole
  testConversion(0.0000, 'E');    // Prime meridian
}

void loop() {
  // Nothing here — one-time test
}

void testConversion(float coordinate, char direction) {
  float decimal = convertDDMMmmToDecimalDegrees(coordinate, direction);
  Serial.print("Raw: ");
  Serial.print(coordinate, 6);
  Serial.print("  Dir: ");
  Serial.print(direction);
  Serial.print("  →  Decimal: ");
  Serial.println(decimal, 6);
}

float convertDDMMmmToDecimalDegrees(float coordinate, char direction) {
  // Extract degrees and minutes
  int degrees = int(coordinate / 100);
  float minutes = coordinate - (degrees * 100);

  float decimalDegrees = degrees + (minutes / 60.0);

  // Apply sign for South/West
  if (direction == 'S' || direction == 'W') {
    decimalDegrees = -decimalDegrees;
  }

  return decimalDegrees;
}
