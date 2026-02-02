
// 8-channel 3A relay output
const int RO1 = 40;
const int RO2 = 41;
const int RO3 = 42;
const int RO4 = 43;
const int RO5 = 44;
const int RO6 = 45;
const int RO7 = 46;
const int RO8 = 47;

//We need to set the pins for two limit sensors in this section
const int limit1 = 0;
const int limit2 = 1;

int id;
String parsedCommand;
int timeToComplete;

class Motor {
  private:
    int RelayPositive;
    int RelayNegative;
    int limitSensor;
    bool isRunning;
  
  public:
    //Constructor
    Motor(int& RelayPositive, int& RelayNegative, int& limitSensor){
      this-> RelayPositive = RelayPositive;
      this-> RelayNegative = RelayNegative;
      this-> limitSensor = limitSensor;
      this-> isRunning = false; //initiallyOff
      
      pinMode(RelayPositive, OUTPUT);
      pinMode(RelayNegative, OUTPUT);
      pinMode(limitSensor,   INPUT);
      
      digitalWrite(RelayPositive, LOW);
      digitalWrite(RelayNegative, LOW);
      }

    void extendActuator(){
      if(digitalRead(limitSensor)==LOW){
        digitalWrite(RelayPositive, HIGH);
        this->isRunning = true;
        }
     }
     
     void retractActuator(){
        digitalWrite(RelayNegative, HIGH);
        this->isRunning = true;
     }

     void stopMotor() {
         digitalWrite(RelayPositive, LOW);
         digitalWrite(RelayNegative, LOW);
         this->isRunning = false;
         Serial.println("motor Opened or Closed\r\n")
      }
     
     void updateMotor() {
         if(isRunning && digitalRead(limitSensor)){
           stopMotor()
          }
      }
 }

class Dispenser{
  private:
    Motor actuatorA;
    Motor actuatorB;
    int id;
    
  public:
     //Constructor
     Dispenser(int& id, Motor& actuatorA, Motor& actuatorB){
      this -> actuatorA = actuatorA;
      this -> actuatorB = actuatorB;
      this -> id        = id;
      }
      
     void openDispenser(){
        actuatorA.extendActuator();
        actuatorB.extendActuator();
      }

     void closeDispenser(){
        actuatorA.retractActuator();
        actuatorB.retractActuator();
      }

     
     void stopDispenser(){
        actuatorA.stopMotor();
        actuatorB.stopMotor();
      }

     void updateDispenser(){
        actuatorA.updateMotor();
        actuatorB.updateMotor();
      }
     
  
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

    //find time part
    startIndex = endIndex + 1;
    timeUnprocessed = receivedCommand.substring(startIndex);
    timeToComplete = timeUnprocessed.toInt();
  }




void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Motor motor2A = Motor(RO2, RO1, limit2);
  Motor motor2B = Motor(RO4, RO3, limit2);

  Motor motor1A = Motor(RO6, RO5, limit1);
  Motor motor1B = Motor(RO8, RO7, limit1);

  Dispenser dispenser1 = Dispenser(1, motor1A, motor1B);
  Dispenser dispenser2 = Dispenser(2, motor2A, motor2B);

  while (Serial.available() > 0) {
    Serial.read();
  }

  void movingDispensers (int id, String command){
    if       (id == 2 && command == "open"){
        dispenser2.openDispenser();
    }
    else if (id == 2 && command == "close"){
        dispenser2.closeDispenser();
    }
    else if (id == 1 && command == "open"){
        dispenser1.openDispenser();
    }
    else if (id == 1 && command == "close"){
        dispenser1.closeDispenser();
    }
    else if (((id == 12)||(id == 21)) && command == "open"){
        dispenser1.openDispenser();
        dispenser2.openDispenser();
    }
    else if (((id == 12)||(id == 21)) && command == "close"){
        dispenser1.closeDispenser();
        dispenser2.closeDispenser();
    }
  
  }


}

void loop() {
  // put your main code here, to run repeatedly:

}