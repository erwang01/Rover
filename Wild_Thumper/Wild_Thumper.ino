#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

// Pin number constants
const int temperaturePin = 5;
const int lightPin = 13;
const int voltageInput = 4;
//long duration, cmA, cmB;
unsigned long lastComTime;
unsigned long timeOfCurrentCheck;
unsigned long lastStatusUpdate;

int motor1Last = 0;
int motor2Last = 0;

String commandlast;

//batteryChecker
const double R1 = 100000.0; // resistance of R1 (100K). R1 is closer to (+) terminal. R1 increases, voltage readin decreases, max voltage of battery increases.
const double R2 = 100000.0; // resistance of R2 (100K). R2 is closer to (-) terminal. R2 increases, voltage readin increases, max voltage of battery decreases.
double totalCurrent = 0;

boolean isConnected = false;

void setup() {
  Serial.begin(9600);
  Serial.flush();
  pinMode(lightPin, OUTPUT);
  md.init();
  lights(true);
  pinMode(voltageInput, INPUT);
  Serial.println("Starting Up");
  lastStatusUpdate = millis();
}

void loop() {
  String command;      //command, pulled from serial
  String motor1;    //pwm of Motor 1
  String motor2;          //pwm of Motor 2
  int index;           // index, placement of "," which separates motor 1 and motor 2
  int motor1Value = 0;
  int motor2Value = 0;

  if (Serial.available() > 0) {     //wait until there are items in buffer
    delay(10);
    Serial.println("reading from buffer");
    command = Serial.readString();    //get commands
    command.toLowerCase();    //change to lower case
    Serial.println(command);
    isConnected = true;
    lastComTime = millis();
    if (command != commandlast && command.indexOf(",") != -1) {
      commandlast = command;
      index = command.indexOf(",");    //find location of ","
      motor1 = getString (command, 0, (index)); // takes all characters in command string between pos0 to pos of index
      motor2 = getString(command,(index+1), command.length()); // takes all caracters in command string from pos of index to end
      motor1.trim();   //remove white spaces
      motor2.trim();
      motor1Value = motor1.toInt();
      motor2Value = motor2.toInt();

      //constrain motor values down to -400 to 400
      motor1Value = constrain(motor1Value, -400, 400);
      motor2Value = constrain(motor2Value, -400, 400);
      
      Serial.print("M1: ");
      Serial.println(motor1Value);
      Serial.print("M2: ");
      Serial.println(motor2Value);

      if (motor1Value == 0 && motor2Value == 0) {
        md.setBrakes(200,200);
        Serial.println("M1: breaks");
        Serial.println("M2: breaks");
      }
      else {
        motorSpeed(motor1Value, motor2Value);
      }
      motor1Last = motor1Value;
      motor2Last = motor2Value;
    }
  }
  
  if (lastComTime <= millis() - 2000) {
    isConnected = false;
    md.setBrakes(200,200);
    lastComTime = millis()-1000;
  }

  if(lastStatusUpdate <= millis() - 500) {
    logStatus();
  }
}

void lights(boolean on) {
  if (on) {
    digitalWrite(lightPin, HIGH);
  }
  else {
    digitalWrite(lightPin, LOW);
  }
}

void logTemperature() {
  float voltage, avgTemp = 0;
  int times = 5;
  for(int i = 0; i < times; i++) {
    voltage = analogRead(temperaturePin) * 0.004882814;
    avgTemp += (voltage - 0.5) *100.0;
  }
  avgTemp /= times;
  Serial.print("Deg C:  ");
  Serial.println(avgTemp);
}


String getString(String stringIn, int startPos, int endPos) {
  String stringOut;
  for (int i = startPos; i< endPos; i++){
    stringOut = stringOut + stringIn.charAt(i);
  }
  return stringOut;
}

//fault detection

void stopIfFault()
{
  if (md.getM1Fault()) {
    Serial.println("M1: fault");
    motorSpeed(0,0);
    while(1);
  }
  if (md.getM2Fault()) {
    Serial.println("M2: fault");
    motorSpeed(0,0);
    while(1);
  }
}

//Setting The Speed

void motorSpeed (int motor1speed, int motor2speed) {
   if (motor1Last == 0 && motor2Last == 0 && motor1speed > 100 && motor2speed > 100) {
    motor1Last += 50;
    motor2Last += 50;
    md.setM1Speed(motor1Last);
    md.setM2Speed(motor2Last);
    delay(1);
    motor1Last += 50;
    motor2Last += 50;
    md.setM1Speed(motor1Last);
    md.setM2Speed(motor2Last);
    delay(1);
   }

   if (motor1Last == 0 && motor2Last == 0 && motor1speed < 100 && motor2speed < 100) {
    motor1Last -= 50;
    motor2Last -= 50;
    md.setM1Speed(motor1Last);
    md.setM2Speed(motor2Last);
    delay(1);
    motor1Last -= 50;
    motor2Last -= 50;
    md.setM1Speed(motor1Last);
    md.setM2Speed(motor2Last);
    delay(1);
   }
   
   //Alternate code
   while(motor1Last != motor1speed || motor2Last != motor2speed) {
     if(motor1Last > motor1speed)
     {
       md.setM1Speed(motor1Last--);
       //Serial.print(motor1Last);
     }
     
     if(motor1Last <  motor1speed) {
       md.setM1Speed(motor1Last++);
       //Serial.print(motor1Last);
     }
     
     if(motor2Last > motor2speed) {
       md.setM2Speed(motor2Last--);
     }
     
     if(motor2Last <  motor2speed) {
       md.setM2Speed(motor2Last++);
     }
     
     stopIfFault();
     if (motor1Last%50 == 1) {
       checkCurrent();
     }
     
     delay(0.5);
   }
}

//Current Checking

void checkCurrent() {
    unsigned int M1Current = md.getM1CurrentMilliamps();
    unsigned int M2Current = md.getM2CurrentMilliamps();
    
    Serial.print("M1 current: ");
    Serial.println(M1Current);
    Serial.print("M2 current: ");
    Serial.println(M2Current);
    
    if (M1Current >= 16000) {
      motorSpeed(0,0);
      Serial.println("Warning");
      Serial.println("M1 Current is over safety of 16 A");
      Serial.println("Warning");
    }
    if (M2Current >= 16000) {
      motorSpeed(0,0);
      Serial.println("Warning");
      Serial.println("M2 Current is over safety of 16 A");
      Serial.println("Warning");
    }
}


//Check battery Status/Percentage
boolean batteryStatus() {
  boolean batteryStatus = true;
  // read the value at analog input
  double vinAvg = 0;
  for (int i = 0; i < 3; i++)
  {
    double vout = (analogRead(voltageInput) * 5.0) / 1024.0; // voltage recieved by analog input.
    double vin = vout / (R2/(R1+R2)); //convert voltage 
    if (vin<0.09) {
      vin=0.0;//statement to quash undesired reading !
    }
    vinAvg += vin;
  }
  vinAvg = vinAvg/3;
  Serial.print("VBatt: ");
  Serial.println(vinAvg);

  //check batteryStatus
  if (vinAvg < 6.6) {
    batteryStatus = false;
  }

  //get current
  unsigned int M1Current = md.getM1CurrentMilliamps();
  unsigned int M2Current = md.getM2CurrentMilliamps();

  unsigned int current = M1Current+M2Current;

  if (current > 15000) {
    batteryStatus = false;
  }

  double timeSinceLastCheck= millis()-timeOfCurrentCheck; //get seconds since last log
  timeSinceLastCheck = timeSinceLastCheck/1000/60/60; //get time in hours
  totalCurrent += current*timeSinceLastCheck; //add to total mAh used
  Serial.print("IBatt: ");
  Serial.println(current);
  Serial.print("CBatt: ");
  Serial.println(totalCurrent);
  timeOfCurrentCheck = millis();
  return batteryStatus;
}

void logStatus() {
    //perform logging
      if (isConnected) {
        Serial.println("Status: connected");
      }
      else {
        Serial.println("Status: disconnected");
      }
      checkCurrent();
      logTemperature();
      batteryStatus();
      lastStatusUpdate = millis();
}

