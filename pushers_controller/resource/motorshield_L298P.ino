int pwmA = 10;
int pwmB = 11;
int enA = 12;
int enB = 13;
int buz = 4;

void setup() {
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(buz, OUTPUT);
  // Initialize Serial Communication
  Serial.begin(9600);
  // Reseting position
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  analogWrite(pwmA, 255); 
  analogWrite(pwmB, 255); 
  Serial.println("Reseting_motor...");
  delay(2500);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  Serial.println("Motor_ready");
  //Serial.println("Enter 'open' or 'close'");
}

void loop() {
  // Check if there's any data available in the Serial buffer
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the command as a string
    command.trim(); // Remove any extra spaces or newline characters
    if (command == "close") {
      digitalWrite(enA, HIGH);
      digitalWrite(enB, HIGH);
      analogWrite(pwmA, 255); 
      analogWrite(pwmB, 255);   
      Serial.println("Motor_is_closing...");
      delay(2500);
      analogWrite(pwmA, 0);
      analogWrite(pwmB, 0);
      Serial.println("Motor_closed");
    }
    else if (command == "open") {
      digitalWrite(enA, LOW);
      digitalWrite(enB, LOW);
      analogWrite(pwmA, 255); 
      analogWrite(pwmB, 255); 
      Serial.println("Motor_is_opening.");
      delay(2500);
      analogWrite(pwmA, 0);
      analogWrite(pwmB, 0);
      Serial.println("Motor_opened");
    }
    else {
      // Handle invalid commands
      Serial.println("Invalid command. Please enter 'on' or 'off'.");
    }
  }
}