#include <AFMotor.h>

AF_DCMotor motor(3);
String InBytes;


void setup() {
  Serial.begin(9600);
 
  motor.setSpeed(200);

  motor.run(BACKWARD);
  delay(10000);
  motor.run(RELEASE);
}

void loop() {
  if (Serial.available()>0) {
      InBytes = Serial.readStringUntil('\n');
      if (InBytes == "pusher3_open"){
        motor.run(FORWARD);
        delay(5000);
        motor.run(RELEASE);
        Serial.println("completed");
        }

        if (InBytes == "pusher3_close"){
        motor.run(BACKWARD);
        delay(5000);
        motor.run(RELEASE);
        Serial.println("completed");
        }
    
    }
}
