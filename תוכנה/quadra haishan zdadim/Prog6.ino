#include <Servo.h>

#define DEBUG1 0 // motor speed left/right
#define DEBUG2 0 // sensors digital values
#define DEBUG3 0 // sensors analog values
#define DEBUG4 0 // sensors analog values RAW
#define DEBUG5 0 // UltraSonic Sensor
#define DEBUG6 0 // Mercury sensor


// Catching Servos
Servo kl,kr;
Servo zl,zr;  

void openZ(){
  kl.write(110);
  kr.write(70);
 }
 
void closeZ(){
  kl.write(70);
  kr.write(100);
 }
 
void moveUp(){
 zl.write(40);
 zr.write(140);
}
 
void moveDown(){
 zl.write(110);
 zr.write(70);
}



// Black Line Value
#define black 250 

// Engine Pin Layout
#define rightMotor_A 6
#define rightMotor_B 7
#define rightMotor_PWM 8
#define leftMotor_A 4
#define leftMotor_B 5
#define leftMotor_PWM 9

// Ultrasonic Sensor
#define trigPin 14
#define echoPin 3
unsigned long time = 0;
unsigned long duration;
int distance = 0;

// Mercury Sensor
#define caspitInput 2
volatile boolean cFlag=false;
boolean inclineFlag = false;
unsigned long caspitTime;

//Infrared Line Sensors
int rightSensorMidRAW,leftSensorMidRAW,rightSensorOutRAW,leftSensorOutRAW;
int rightSensorMid,leftSensorMid,rightSensorOut,leftSensorOut;
int rightSub,leftSub;

boolean digitalSensor[4];
int analogSensor[4];
int analogSensorRAW[4];
const char* sensorNum[] = {"rightSensorOut(A2)" , "rightSensorMid(A0)" , "leftSensorMid(A3)" , "leftSensorOut(A1)"};


void trigUs(){
  digitalWrite(trigPin, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);
  duration = pulseIn(echoPin,HIGH);
  attachInterrupt(1, measurement, RISING);
}

void measurement(){
  distance = duration/58;
  
  if(DEBUG5){
    Serial.print("Distance = ");
    Serial.println(distance);
  }
  detachInterrupt(1);  
}


void stopingz(){
  motors(0,0);
}

void readSensors(){
  rightSensorMidRAW = analogRead(A0);
  leftSensorMidRAW = analogRead(A3);
  rightSensorOutRAW = analogRead(A2);
  leftSensorOutRAW =analogRead(A1);

  rightSensorOut = map(rightSensorOutRAW, 1000, 110, 1000, 0);
  rightSensorMid = map(rightSensorMidRAW,1000 ,115, 1000, 0);
  leftSensorMid = map(leftSensorMidRAW, 1000, 190,  1000, 0);
  leftSensorOut = map(leftSensorOutRAW, 1000, 120,1000, 0);
  
   analogSensor[0] = rightSensorOut;
   analogSensor[1] = rightSensorMid;
   analogSensor[2] = leftSensorMid;
   analogSensor[3] = leftSensorOut;

   analogSensorRAW[0] = rightSensorOutRAW;
   analogSensorRAW[1] = rightSensorMidRAW;
   analogSensorRAW[2] = leftSensorMidRAW;
   analogSensorRAW[3] = leftSensorOutRAW;
    }
    
void updateDigital() {
  for(int i = 0; i< 4; i++){
    if(analogSensor[i] < black)
        digitalSensor[i] = true;
    else
        digitalSensor[i] = false;   
  }
}  


void updateSensors(){
  readSensors();
  updateDigital();
  caspitState();
   
   
  if(millis() - time > 50){
    trigUs();
    time = millis();
  }
  
  
  if(DEBUG2){
    for(int c = 0; c < 4; c++)
    {
    for(int i = 0; i< 4; i++){
        Serial.print(sensorNum[i]);
        Serial.print(" = ");
        Serial.print(digitalSensor[i]);
         Serial.print("  ");
      }
      Serial.println();
    }
    
    }
 
 if(DEBUG3){
   for(int c = 0; c < 4;c++)
    for(int j = 0; j< 4; j++){
        Serial.print(sensorNum[j]);
        Serial.print(" = ");
        Serial.print(analogSensor[j]);
        Serial.print("  ");
    }
    Serial.println();
 }
  
 
if(DEBUG4){
  for( int c = 0; c < 4; c++)
  {
   for(int k = 0; k < 4; k++){
        Serial.print(sensorNum[k]);
        Serial.print(" = ");
        Serial.print(analogSensorRAW[k]);
        Serial.print("  ");
    }
    Serial.println();
  }
  }
  
  
}

void caspit_init(){
  pinMode(caspitInput, INPUT_PULLUP);
  attachInterrupt(0, setCaspitFlag, CHANGE);
}

void setCaspitFlag(){
  cFlag = true;
}    

boolean caspitState(){ 
  if(cFlag){
    if(digitalRead(caspitInput)==0){
    caspitTime=millis();
    }
  }
   else if(digitalRead(caspitInput)==0)
    {
        if((millis()-caspitTime)>50){
          if(DEBUG6)
              Serial.println("CaspitState : true");
          return true;
        }
    }
    
    if(DEBUG6)
      Serial.println("CaspitState = false");
    
    cFlag=false;
    return false;
  }
 

void inclineCapot(){
  updateSensors();
  if(inclineFlag)
    moveDown;
  if(inclineFlag && distance < 20){
    moveUp();
    inclineFlag = false;
  }
      
}
 
void Drive(){
  inclineCapot();

  if(!caspitState()){
    if((leftSensorOut < black) || (rightSensorOut < black))
      turnAround(); 

    else
       drive1(60,60);
     
  }
  else{
    inclineFlag = true;
    DaniBoost(200,200);  
  }
} 
    
void drive1(int speed1,int speed2){
  
  int rightSensorMid1 = map(rightSensorMid, 0, 1000, -100, 100);
  int leftSensorMid1 = map(leftSensorMid, 0, 1000, -100, 100);
  int rightSensorOut1 = map(rightSensorOut, 0, 1000, -100, 100);
  int leftSensorOut1 = map(leftSensorOut, 0, 1000, -100, 100);
  
  int error = (rightSensorMid1 - leftSensorMid1);
   
  motors(speed1 + error,speed2 - error);
}

void DaniBoost(int speed3,int speed4){
  
  int rightSensorMid1 = map(rightSensorMid, 0, 1000, 0, 60);
  int leftSensorMid1 = map(leftSensorMid, 0, 1000, 0, 60);
  
  int error = (rightSensorMid1 - leftSensorMid1);
  
  motors(speed3 + error,speed4 - error); 
}


void turnAround(){
    if(leftSensorOut < black){
      motors(60,-60);
      delay(50);
        while(rightSensorMid > black){
              updateSensors();
        }
      }
     else if(rightSensorOut < black){
       motors(-60 ,60);
       delay(50);
        while(leftSensorMid > black){
            updateSensors();   
        }
     }
}

void motors(int left, int right){
  if(DEBUG1){
     Serial.print("Left: ");
     Serial.print(left);
     Serial.print("Right: ");
     Serial.println(right);  
  }
 
  if (left > 0)
  {
    digitalWrite(leftMotor_A,1);
    digitalWrite(leftMotor_B,0);  
    analogWrite(leftMotor_PWM, left);
  }
  else
  {
    digitalWrite(leftMotor_A,0);
    digitalWrite(leftMotor_B,1);  
    analogWrite(leftMotor_PWM, -left);
  }

  if (right > 0)
  {
    digitalWrite(rightMotor_A,1);
    digitalWrite(rightMotor_B,0);  
    analogWrite(rightMotor_PWM, right);
  }
  else
  {
    digitalWrite(rightMotor_A,0);
    digitalWrite(rightMotor_B,1);
    analogWrite(rightMotor_PWM, -right);
  }
}


void setup(){
  Serial.begin(9600);
  for(int i = 4; i < 8; i ++)
    pinMode(i,OUTPUT);

  pinMode(trigPin,OUTPUT);   
  trigUs();
  caspit_init();

  kr.attach(43); 
  kl.attach(45); 
  zl.attach(44); 
  zr.attach(42); 

  moveUp();
}

void loop(){
  updateSensors();
  Drive();
}
