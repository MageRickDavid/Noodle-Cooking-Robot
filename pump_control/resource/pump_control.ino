#define RELAY_1 2
#define RELAY_2 3
#define RELAY_3 4
#define RELAY_4 5
#define RELAY_5 6
#define RELAY_6 7
#define RELAY_7 8
#define RELAY_8 9
#define BUILTIN_LED LED_BUILTIN  // Define the built-in LED pin

bool relayStates[8] = {false, false, false, false, false, false, false, false};  // Stores relay states (OFF by default)

void setup() {
  // Set all relay pins as OUTPUT
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
  pinMode(RELAY_5, OUTPUT);
  pinMode(RELAY_6, OUTPUT);
  pinMode(RELAY_7, OUTPUT);
  pinMode(RELAY_8, OUTPUT);



  delay(10000);

  // Set the built-in LED as OUTPUT
  pinMode(BUILTIN_LED, OUTPUT);

  // Initialize all relays and built-in LED as OFF
  for (int i = 0; i < 8; i++) {
    digitalWrite(RELAY_1 + i, HIGH);  // Set all relays to OFF (active-low)
  }
  digitalWrite(BUILTIN_LED, LOW);

  // Start Serial Communication
  Serial.begin(9600);
  Serial.println("Ready to receive commands! Press 1-4 to control relays, 9 for built-in LED ON, -9 for OFF.");
}

void loop() {
  // Check if data is available from the serial monitor
  if (Serial.available() > 0) {
    int command = Serial.parseInt();  // Read the command as an integer
    Serial.println("Command received: " + String(command));

    // Process the command
    if (command > 0 && command <= 4) {
      controlSet(command, true);  // Turn on the corresponding set
      Serial.println("Set " + String(command) + " turned ON.");
    } else if (command < 0 && command >= -4) {
      controlSet(-command, false);  // Turn off the corresponding set
      Serial.println("Set " + String(-command) + " turned OFF.");
    } else {
      Serial.println("Invalid command! Use 1-4 to control relays, 9 for built-in LED ON, -9 for OFF.");
    }
  }
}

// Function to control a specific set of relays
void controlSet(int setNumber, bool turnOn) {
  // Determine the relay pins for the given set
  int relay1, relay2;

  switch (setNumber) {
    case 1:
      relay1 = RELAY_1;
      relay2 = RELAY_2;
      break;
    case 2:
      relay1 = RELAY_3;
      relay2 = RELAY_4;
      break;
    case 3:
      relay1 = RELAY_5;
      relay2 = RELAY_6;
      break;
    case 4:
      relay1 = RELAY_7;
      relay2 = RELAY_8;
      break;
    default:
      // Invalid set number
      return;
  }

  // Update relay states
  int index1 = relay1 - RELAY_1;
  int index2 = relay2 - RELAY_1;
  relayStates[index1] = turnOn;
  relayStates[index2] = turnOn;

  // Turn the relays ON or OFF based on the state
  digitalWrite(relay1, turnOn ? LOW : HIGH);  // LOW = ON, HIGH = OFF
  digitalWrite(relay2, turnOn ? LOW : HIGH);
}