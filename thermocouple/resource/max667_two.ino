#include "max6675.h"

// Define pins for sensor 1
const int CS_1 = 10;
const int S0_1 = 9;

// Define pins for sensor 2
const int CS_2 = 12;
const int S0_2 = 11;

// Shared clock pin
const int SLCK = 13;

// Create MAX6675 sensor objects
MAX6675 sensor_1(SLCK, CS_1, S0_1);
MAX6675 sensor_2(SLCK, CS_2, S0_2);

void setup() {
  Serial.begin(9600);
  Serial.println("MAX6675 Temperature Sensor Test");

  // Print table headers with proper spacing
  Serial.println("C1 (°C)\t\tC2 (°C)");

  // Wait for sensors to stabilize
  delay(500);
}

void loop() {
  // Read temperatures from the sensors
  float temp1 = sensor_1.readCelsius();
  float temp2 = sensor_2.readCelsius();

  // Print temperatures in tabular format
  Serial.print(temp1, 2); // Print temperature from sensor 1 with 2 decimal places
  Serial.print("\t\t");   // Add a tab for separation
  Serial.print(temp2, 2); // Print temperature from sensor 2 with 2 decimal places
  Serial.println();       // Move to the next line

  // Delay to allow MAX6675 to update
  delay(500);
}