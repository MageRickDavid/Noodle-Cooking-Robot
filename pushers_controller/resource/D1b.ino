#include <Wire.h>

const int pwmA = 10; // PWM pin for motor control
const int enA = 12;  // Direction control pin
const int buz = 4;   // Buzzer pin (if used for feedback)

byte motorCommand = 0; // 0 = close, 1 = open

void setup() {
  pinMode(pwmA, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(buz, OUTPUT);
  
  // Initialize I2C as a slave with address 1
  Wire.begin(1);  // 2 is the slave address
  Wire.onReceive(receiveData); // Set function to call when data is received
  
  // Initialize Serial Communication for debugging (optional)
  Serial.begin(9600);
  
  // Resetting position
  digitalWrite(enA, HIGH);
  analogWrite(pwmA, 255); 
  Serial.println("Reseting_motor...");
  delay(2500); // Wait for reset operation
  analogWrite(pwmA, 0);
  Serial.println("Motor_ready");
}

void loop() {
  // Main loop does nothing but wait for I2C data
  delay(100);
}

void receiveData(int numBytes) {
  if (numBytes > 0) {
    motorCommand = Wire.read(); // Read the command byte
    Serial.print("Received Command: ");
    Serial.println(motorCommand);

    if (motorCommand == 0) {
      closeMotor();
    } else if (motorCommand == 1) {
      openMotor();
    } else {
      Serial.println("Invalid Command");
    }
  }
}

void openMotor() {
  Serial.println("Motor_is_opening.");
  for (int i = 0; i < 10; i++) { // Adjust number of steps as needed
    digitalWrite(enA, LOW);      // Enable motor in the opening direction
    analogWrite(pwmA, 255);      // Full speed
    delay(35000);                  // Run motor for 0.3 seconds
    analogWrite(pwmA, 0);        // Stop motor
    delay(500);                  // Wait for 0.2 seconds
  }
  Serial.println("Motor_opened");
}

void closeMotor() {
  Serial.println("Motor_is_closing...");
  for (int i = 0; i < 10; i++) { // Adjust number of steps as needed
    digitalWrite(enA, HIGH);     // Enable motor in the closing direction
    analogWrite(pwmA, 255);      // Full speed
    delay(70000);                  // Run motor for 0.3 seconds
    analogWrite(pwmA, 0);        // Stop motor
    delay(500);                  // Wait for 0.2 seconds
  }
  Serial.println("Motor_closed");
}
