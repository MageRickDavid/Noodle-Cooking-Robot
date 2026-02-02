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

int id;
String parsedCommand;
int timeToComplete;

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

//  Serial.println("System initialized. Enter commands to control the Dispensors:");
//  Serial.println("d1_open - Open Dispensor 1");
//  Serial.println("d1_close - Close Dispensor 1");
//  Serial.println("d2_open - Open Dispensor 2");
//  Serial.println("d2_close - Close Dispensor 2");
//  Serial.println("d12_open - Open Dispensor 12");
//  Serial.println("d12_close - Close Dispensor 12");
}
void string_process(String& receivedCommand, int& id, String& parsedCommand, int& timeToComplete){
  int startIndex = 0;
  int endIndex = receivedCommand.indexOf("_");
  String idUnprocessed;
  String timeUnprocessed;
  
  //find id part
  idUnprocessed = receivedCommand.substring(startIndex, endIndex);
  id = idUnprocessed.toInt();

  //find command part
  startIndex = endIndex + 1;
  endIndex = receivedCommand.indexOf('_', startIndex);
  parsedCommand = receivedCommand.substring(startIndex, endIndex);

//  if id < 10{
      //find time part
      startIndex = endIndex + 1;
      timeUnprocessed = receivedCommand.substring(startIndex);
      timeToComplete = timeUnprocessed.toInt();
 //   }
//   else {
//    //find time part
//    startIndex = endIndex + 1;
//    endIndex = receivedCommand.indexOf('_', startIndex);
//    timeUnprocessed = receivedCommand.substring(startIndex, endIndex);  
//    timeToComplete = timeUnprocessed.toInt();
//
//    //find proportional part
//    }
}

void movingActuators(int id, String command, int timeToComplete){
  if (id == 2 && command == "open"){
        digitalWrite(RO2, HIGH); 
        digitalWrite(RO4, HIGH); 
        //Serial.println("Opening Dispensor 2");
        
        if (timeToComplete <=0) {
          delay(2200);
             } 
        else {
            delay(timeToComplete);    
              }
        digitalWrite(RO2, LOW); 
        digitalWrite(RO4, LOW); 
        Serial.println("Dispensor 2 is Opened\r\n");
    }

  else if (id == 2 && command == "close") {
        digitalWrite(RO1, HIGH); 
        digitalWrite(RO3, HIGH); 
        //Serial.println("Closing Dispensor 2");

       if (timeToComplete <=0) {
          delay(2200);
             } 
        else {
            delay(timeToComplete);    
              }

        digitalWrite(RO1, LOW); 
        digitalWrite(RO3, LOW); 
        Serial.println("Dispensor 2 Closed\r\n");
    }

   else if (id == 1 && command == "open"){
      
        digitalWrite(RO6, HIGH); 
        digitalWrite(RO8, HIGH); 
        //Serial.println("Opening Dispensor 1");
        
        if (timeToComplete <=0) {
          delay(1700);
          }
          else {
            delay(timeToComplete);
            }
        digitalWrite(RO6, LOW); 
        digitalWrite(RO8, LOW); 
        Serial.println("Dispensor 1 is Opened\r\n");
      }
   else if (id == 1 && command == "close") {
      
        digitalWrite(RO5, HIGH); // Turn on Relay 1
        digitalWrite(RO7, HIGH); // Turn on Relay 3
        //Serial.println("Closing Dispensor 1");

       if (timeToComplete <=0) {
          delay(1700);
          }
          else {
            delay(timeToComplete);
            }

        digitalWrite(RO5, LOW); // Turn off Relay 1
        digitalWrite(RO7, LOW); // Turn off Relay 3
        Serial.println("Dispensor 1 is Closed\r\n");
      
      }
     else if (((id == 12)||(id == 21)) && command == "open"){
       // open dispenser 1 it needs to stop at 1700
        digitalWrite(RO6, HIGH); 
        digitalWrite(RO8, HIGH);

        // open dispenser 2 it needs to stop at 2200
        digitalWrite(RO2, HIGH); 
        digitalWrite(RO4, HIGH); 

        // stop dispenser 1
         if (timeToComplete <=0) {
          delay(1700);
          }
          else {
            delay(static_cast<int>(timeToComplete*0.908));
            }
        digitalWrite(RO6, LOW); 
        digitalWrite(RO8, LOW);

        // stop dispenser 2
        if (timeToComplete <= 0){
          delay(150);
          }
         else {
          delay(static_cast<int>(timeToComplete*0.092));
          }
        digitalWrite(RO2, LOW); 
        digitalWrite(RO4, LOW);

        Serial.println("Dispensor 12 is Opened\r\n"); 
      }
    else if (((id==12)||(id==21)) && command == "close"){
      // close dispenser 1 it needs to stop at 1700
          digitalWrite(RO5, HIGH); // 
          digitalWrite(RO7, HIGH); // 
          
          // close dispenser 2 it needs to stop at 2200
          digitalWrite(RO1, HIGH); // 
          digitalWrite(RO3, HIGH); //
          
          // stop dispenser 1
          if (timeToComplete<=0){
            delay(1700);
            }
          else {
            delay(static_cast<int>(timeToComplete*0.77));
            }
           
          digitalWrite(RO5, LOW); // 
          digitalWrite(RO7, LOW); //

          // stop dispenser 2
          if (timeToComplete<=0){
            delay(500);
            }
          else {
            delay(static_cast<int>(timeToComplete*0.23));
            }
          digitalWrite(RO1, LOW); // 
          digitalWrite(RO3, LOW); //
          Serial.println("Dispensor 12 is Closed\r\n");
      }
       else {
        Serial.println("Unknown command");
      }
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
      string_process(command, id, parsedCommand, timeToComplete);
      movingActuators(id, parsedCommand, timeToComplete);
      

      // Clear the command for the next input
      command = "";
    } else {
      command += incomingByte; // Append incoming character to command
    }
  }
}