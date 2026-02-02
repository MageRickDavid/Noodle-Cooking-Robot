#define uchar unsigned char
#define uint unsigned int

// 8-channel 3A relay output
const int RO1 = 40;
const int RO2 = 41;
const int RO3 = 42;
const int RO4 = 43;
const int RO5 = 44;
const int RO6 = 45;
const int RO7 = 46;
const int RO8 = 47;

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Set relay pins as OUTPUT
  pinMode(RO1, OUTPUT);
  pinMode(RO2, OUTPUT);
  pinMode(RO3, OUTPUT);
  pinMode(RO4, OUTPUT);
  pinMode(RO5, OUTPUT);
  pinMode(RO6, OUTPUT);
  pinMode(RO7, OUTPUT);
  pinMode(RO8, OUTPUT);

  // Initialize all relays to LOW (off)
  digitalWrite(RO1, LOW);
  digitalWrite(RO2, LOW);
  digitalWrite(RO3, LOW);
  digitalWrite(RO4, LOW);
  digitalWrite(RO5, LOW);
  digitalWrite(RO6, LOW);
  digitalWrite(RO7, LOW);
  digitalWrite(RO8, LOW);

  // Clear serial buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("System initialized. Enter commands to control Dispensors:");
  Serial.println("d3_open - Open Dispensor 3");
  Serial.println("d3_close - Close Dispensor 3");
  Serial.println("d4_open - Open Dispensor 4");
  Serial.println("d4_closed - Close Dispensor 4");
}

void loop() {
  static String command = ""; // Accumulate incoming commands

  // Check if serial data is available
  while (Serial.available() > 0) {
    char incomingByte = Serial.read(); // Read one character
    if (incomingByte == '\n') {       // End of command
      command.trim();                 // Remove leading/trailing whitespace

      // Debug: Print the received command
      //Serial.print("Received command: ");
      //Serial.println(command);

      // Handle specific commands
      if (command == "d4_open") {
        digitalWrite(RO1, HIGH); // Turn on Relay 1
        digitalWrite(RO3, HIGH); // Turn on Relay 3
        Serial.println("Opening Dispenser 4");

        delay(1900); // Wait for 1 second

        digitalWrite(RO1, LOW); // Turn off Relay 1
        digitalWrite(RO3, LOW); // Turn off Relay 3
        Serial.println("Dispenser 4 Opened");
      } 
      
      else if (command == "d4_close") {
        digitalWrite(RO2, HIGH); 
        digitalWrite(RO4, HIGH); 
        Serial.println("Closing Dispenser 4");
        delay(2200);
        digitalWrite(RO2, LOW); 
        digitalWrite(RO4, LOW); 
        Serial.println("Dispensor 4 is Closed");
      }

      if (command == "d3_open") {
        digitalWrite(RO5, HIGH); // Turn on Relay 1
        digitalWrite(RO7, HIGH); // Turn on Relay 3
        //Serial.println("Opening Dispensor 3");

        delay(2000); // Wait for 2 second

        digitalWrite(RO5, LOW); // Turn off Relay 1
        digitalWrite(RO7, LOW); // Turn off Relay 3
        Serial.println("Dispensor 3 is Opened");
      } 
      
      else if (command == "d3_close") {
        digitalWrite(RO6, HIGH); 
        digitalWrite(RO8, HIGH); 
        //Serial.println("Closing Dispenser 3");
        delay(1800);
        digitalWrite(RO6, LOW); 
        digitalWrite(RO8, LOW); 
        Serial.println("Dispensor 3 is Closed");
      }
     
      else {
        Serial.println("Unknown command");
      }

      // Clear the command for the next input
      command = "";
    } else {
      command += incomingByte; // Append incoming character to command
    }
  }
}