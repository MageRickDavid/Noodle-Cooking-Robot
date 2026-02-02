#include <Wire.h>

const int RPWM = 3;
const int LPWM = 4;

byte Speed = 0;
byte I2C_OnOff = 0;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  Wire.begin(2);
  Wire.onReceive(receiveData);

  Serial.begin(9600);
  Serial.println("I2C Slave2  Initialized");
}

void loop() {
  delay(100);
}

void receiveData(int numBytes) {
  if (numBytes > 0) {
    I2C_OnOff = Wire.read();
    Serial.print("Received Command: ");
    Serial.println(I2C_OnOff);

    if (I2C_OnOff == 1) {
      extendActuator();
    } else if (I2C_OnOff == 0) {
      retractActuator();
    } else {
      Serial.println("Invalid Command");
    }
  }
}

void extendActuator() {
  Speed = 255;
  analogWrite(RPWM, 0);
  analogWrite(LPWM, Speed);
  Serial.println("Extending Actuator");
  delay(2000);
  stopActuator();
}

void retractActuator() {
  Speed = 255;
  analogWrite(RPWM, Speed);
  analogWrite(LPWM, 0);
  Serial.println("Retracting Actuator");
  delay(2000);
  stopActuator();
}

void stopActuator() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  Serial.println("Actuator Stopped");
}
