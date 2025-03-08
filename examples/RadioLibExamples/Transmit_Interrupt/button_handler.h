#include <Arduino.h>

// Define your pin and debounce delay
volatile bool buttonPressed = false;   // Flag set by the interrupt
unsigned long lastButtonPressTime = 0;   // Time of the last valid button press
const unsigned long debounceDelay = 300; // 300 ms debounce delay

void IRAM_ATTR handleButtonInterrupt() {
  // Get current time
  unsigned long currentTime = millis();
  // Check for debounce: only register if enough time has passed since last press
  if (currentTime - lastButtonPressTime >= debounceDelay) {
    buttonPressed = true;
    lastButtonPressTime = currentTime;
  }
}

