/* -------------------------------------------------
Copyright (c)
Arduino project by Tech Talkies YouTube Channel.
https://www.youtube.com/@techtalkies1
-------------------------------------------------*/

#include <Arduino.h>
#include <ESP8266SAM.h>
#include "AudioOutputI2S.h"  // For ESP32-S3 I2S output

// Output object for I2S DAC (MAX98357A)
AudioOutputI2S* out;

// Flag to indicate if text should be spoken
bool speak = false;
String message;

bool printDebug = false;

// I2S Pinout for MAX98357A DAC
#define I2S_BCLK_PIN   26   // Bit Clock Pin (BCLK)
#define I2S_LRCLK_PIN   25   // Word Select Pin (LRCLK)
#define I2S_DIN_PIN     22   // Data Input Pin (DIN)

void setup() {
  if (printDebug)
    Serial.begin(115200);  // Initialize Serial communication for debugging

  // Initialize AudioOutputI2S for ESP32-S3
  out = new AudioOutputI2S();

  // Set the I2S pins
  out->SetPinout(I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DIN_PIN);

  // Set the output gain for the I2S signal
  out->SetGain(2);  // Adjust gain as per your needs

  // Wait for serial input to get the message
  Serial.println("Enter text to speak:");
}

// Main loop where the program runs continuously
void loop() {
  // Check if there is data available from the serial input
  if (Serial.available() > 0) {
    // Read the input string from the serial monitor
    message = Serial.readStringUntil('\n');
    message.trim();  // Remove any extra whitespace or newline characters

    // If there's valid text input, trigger the speaking process
    if (message.length() > 0) {
      if (printDebug) {
        Serial.print("Speaking: ");
        Serial.println(message);
      }
      speak = true;
    }
  }

  // If 'speak' is true, then say the message
  if (speak) {
    ESP8266SAM* sam = new ESP8266SAM;
    sam->Say(out, message.c_str());
    delete sam;  // Free memory after speaking
    speak = false;  // Reset flag after speaking
  }

  // Optionally, you can print the status for debugging
  if (printDebug) {
    Serial.println("Waiting for text input...");
  }
}
