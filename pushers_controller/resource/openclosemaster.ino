#include <Wire.h>

const int slaveAddress1 = 1; // Address of the first I2C slave device
const int slaveAddress2 = 2; // Address of the second I2C slave device

void setup() {
  Wire.begin(); // Initialize I2C as Master
  Serial.begin(9600); // Initialize Serial Communication
  Serial.println("I2C Master Initialized");
  Serial.println("Enter 'open' to extend actuators, 'close' to retract actuators.");
}

void loop() {
  // Check if data is available on the serial monitor
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command

    command.trim(); // Remove any trailing whitespace

    if (command == "open") {
      sendCommandToSlaves("open"); // Send command to extend actuators
    } else if (command == "close") {
      sendCommandToSlaves("close"); // Send command to retract actuators
    } else {
      Serial.println("Invalid input. Enter 'open' to extend or 'close' to retract.");
    }
  }
}

void sendCommandToSlaves(const String &cmd) {
  byte cmdByte = (cmd == "open") ? 1 : 0; // Convert "open" to 1 and "close" to 0

  // Send command to the first slave
  Wire.beginTransmission(slaveAddress1); // Start communication with Slave 1
  Wire.write(cmdByte);                   // Send the command
  Wire.endTransmission();                // End communication

  // Send command to the second slave
  Wire.beginTransmission(slaveAddress2); // Start communication with Slave 2
  Wire.write(cmdByte);                   // Send the command
  Wire.endTransmission();                // End communication

  if (cmd == "open") {
    Serial.println("Command Sent: Extend Actuators on both slaves");
  } else if (cmd == "close") {
    Serial.println("Command Sent: Retract Actuators on both slaves");
  }
}
