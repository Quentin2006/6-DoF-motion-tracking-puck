void setup() {
  // No setup required for built-in RGB driver
}

void loop() {
  // Red color for 1 second
  rgbLedWrite(RGB_BUILTIN, 100, 0, 0);
  delay(1000);

  // Green color for 1 second
  rgbLedWrite(RGB_BUILTIN, 0, 100, 0);
  delay(1000);

  // Blue color for 1 second
  rgbLedWrite(RGB_BUILTIN, 0, 0, 100);
  delay(1000);
}
