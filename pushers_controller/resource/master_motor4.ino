#include <Wire.h>

String InBytes;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Master Initialized");
}

void loop() {
     if (Serial.available()>0) {
      InBytes = Serial.readStringUntil('\n');

        if (InBytes == "pusher1_open"){
        sendCommand(1,1);
        }

        if (InBytes == "pusher1_close"){
        sendCommand(1,0);
        Serial.println("completed");
        }
        

        if (InBytes == "pusher2_open"){
        sendCommand(2,1);
        }

        if (InBytes == "pusher2_close"){
        sendCommand(2,0);
        Serial.println("completed");
        }

        if (InBytes == "pusher3_open"){
        sendCommand(3,1);
        }

        if (InBytes == "pusher3_close"){
        sendCommand(3,0);
        Serial.println("completed");
        }



    
    }
}

void sendCommand(int slaveID, byte cmd) {
  Wire.beginTransmission(slaveID); // Address of the I2C slave
  Wire.write(cmd);
  int result = Wire.endTransmission();
  if (result == 0) {
    Serial.print("Command sent to Slave ");
    Serial.print(slaveID);
    Serial.print(": ");
    Serial.println(cmd == 1 ? "On" : "Off");
  } else {
    Serial.print("Error: Transmission to Slave ");
    Serial.print(slaveID);
    Serial.println(" Failed!");
  }
}