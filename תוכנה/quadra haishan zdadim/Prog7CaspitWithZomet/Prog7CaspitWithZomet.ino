#include <Servo.h>
#include <Encoder.h>
#include <PinChangeInt.h>

#define DEBUG1 0 // motor speed left/right
#define DEBUG2 0 // sensors digital values
#define DEBUG3 0 // sensors analog values
#define DEBUG4 0 // sensors analog values RAW
#define DEBUG5 0 // UltraSonic Sensor
#define DEBUG6 0 // Mercury sensor
#define DEBUG7 0 // 


// Black Line Value
#define black 250 

// Encoders
#define CPR 2248.86
const int M1_ENC_A = 18;
const int M1_ENC_B = 19;
const int M2_ENC_A = 20;
const int M2_ENC_B = 21;
Encoder M1(M1_ENC_A, M1_ENC_B);
Encoder M2(M2_ENC_A, M2_ENC_B);
double M1Spin, M2Spin;

// Servos
const int arm_left =  44, arm_right =  42;
const int grip_left = 45, grip_right = 43;
Servo AL,GL,AR,GR; //Arm left, Grip left, Arm right, Grip right

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
#define caspitInclineInput A8
#define caspitDeclineInput A9
volatile boolean cFlag=false;
volatile boolean cInclineFlag=false;
volatile boolean cDeclineFlag=false;
unsigned long caspitTime;
unsigned long caspitInclineTime;
unsigned long caspitDeclineTime;


// Green Color Sensors
#define rightGreenSensor A10
#define leftGreenSensor A11
boolean ifRightGreenSensor, ifLeftGreenSensor;

// Infrared Line Sensors
int rightSensorMidRAW,leftSensorMidRAW,midSensorRAW,rightSensorOutRAW,leftSensorOutRAW;
int rightSensorMid,leftSensorMid,midSensor,rightSensorOut,leftSensorOut;
int rightSub,leftSub;

boolean digitalSensor[5];
int analogSensor[5];
int analogSensorRAW[5];
const char* sensorNum[] = {"rightSensorOut(A0)" , "rightSensorMid(A1)" , "midSensor(A2)" , "leftSensorMid(A3)" , "leftSensorOut(A4)"};

//Encoder Functions
void MotorSpins(){
  M1Spin = M1.read() /CPR;
  M2Spin = M2.read() /CPR;
}

//Servos Functions
void moveUp(){
  AR.write(140);
  AL.write(40);
  
}

void moveDown(){
  AL.write(110);
  AR.write(70);
}

void openGrip(){
  GR.write(50);
  GL.write(130);
  
}

void closeGrip(){
  GL.write(70);
  GR.write(110);
}

void servo_init(){
  moveUp();
//  closeGrip();
}

/*
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
*/

void stopingz(){
  motors(0,0);
}

void readSensors(){
  rightSensorOutRAW = analogRead(A0);
  rightSensorMidRAW = analogRead(A1);
  midSensorRAW = analogRead(A2);
  leftSensorMidRAW = analogRead(A3);
  leftSensorOutRAW =analogRead(A4);

  rightSensorOut = map(rightSensorOutRAW, 1000, 110, 1000, 0);
  rightSensorMid = map(rightSensorMidRAW,1000 ,115, 1000, 0);
  midSensor = map(midSensorRAW,1000 ,110, 1000, 0);
  leftSensorMid = map(leftSensorMidRAW, 1000, 190,  1000, 0);
  leftSensorOut = map(leftSensorOutRAW, 1000, 120,1000, 0);

  analogSensor[0] = rightSensorOut;
  analogSensor[1] = rightSensorMid;
  analogSensor[2] = midSensor;
  analogSensor[3] = leftSensorMid;
  analogSensor[4] = leftSensorOut;

  analogSensorRAW[0] = rightSensorOutRAW;
  analogSensorRAW[1] = rightSensorMidRAW;
  analogSensorRAW[2] = midSensorRAW;
  analogSensorRAW[3] = leftSensorMidRAW;
  analogSensorRAW[4] = leftSensorOutRAW;
}

void updateDigital() {
  for(int i = 0; i< 5; i++){
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
  MotorSpins();
  greenColorSensorState();

/*  if(millis() - time > 50){
    trigUs();
    time = millis();
  }
*/

  if(DEBUG2){
    for(int c = 0; c < 5; c++)
    {
      for(int i = 0; i< 5; i++){
        Serial.print(sensorNum[i]);
        Serial.print(" = ");
        Serial.print(digitalSensor[i]);
        Serial.print("  ");
      }
      Serial.println();
    }

  }

  if(DEBUG3){
    for(int c = 0; c < 5;c++)
      for(int j = 0; j< 5; j++){
        Serial.print(sensorNum[j]);
        Serial.print(" = ");
        Serial.print(analogSensor[j]);
        Serial.print("  ");
      }
    Serial.println();
  }


  if(DEBUG4){
    for( int c = 0; c < 5; c++)
    {
      for(int k = 0; k < 5; k++){
        Serial.print(sensorNum[k]);
        Serial.print(" = ");
        Serial.print(analogSensorRAW[k]);
        Serial.print("  ");
      }
      Serial.println();
    }
  }

}

void greenColorSensorState(){
  ifRightGreenSensor = digitalRead(rightGreenSensor);
  ifLeftGreenSensor = digitalRead(leftGreenSensor);
}

void colorSensor_init(){
  attachPinChangeInterrupt(A10, greenColorSensorState, CHANGE);
  attachPinChangeInterrupt(A11, greenColorSensorState, CHANGE);
}

void caspit_init(){
  pinMode(caspitInclineInput, INPUT_PULLUP);
   pinMode(caspitDeclineInput, INPUT_PULLUP);
  attachPinChangeInterrupt(A8, setCaspitFlag, CHANGE);
  attachPinChangeInterrupt(A9, setCaspitFlag, CHANGE);
}

void setCaspitFlag(){
  cFlag = true;
}    


void caspitCapot(){
  if(cFlag){
    if(digitalRead(caspitInclineInput)==0)
      caspitInclineTime=millis();
    if(digitalRead(caspitDeclineInput)==0)
      caspitDeclineTime=millis();
 
    }
  
  else if(digitalRead(caspitInclineInput)==0){
    if((millis()-caspitInclineTime)>500){
      cInclineFlag =  1;
       moveDown();
    }
    else{
      caspitInclineTime = millis();
      cInclineFlag = 0;
    }
        }
  else if(digitalRead(caspitDeclineInput)==0){
    if((millis()-caspitDeclineTime)>5000){
        cDeclineFlag = 1;
        Serial.print("0");
         }
        else{
          caspitDeclineTime = millis();
          cDeclineFlag = 0;
        }
      }
      
        if((cDeclineFlag==1) && (digitalRead(caspitDeclineInput)==1 )){
           Serial.print("1");
            moveUp();
              cDeclineFlag = 0;
              cInclineFlag = 0;
              //dec =ירידה
              //inc =עליה
                    }
    
  cFlag=false;
}


boolean caspitState(){ 
  if(cFlag){
    if(digitalRead(caspitInclineInput)==0){
      caspitTime=millis();
    }
  }
  else if(digitalRead(caspitInclineInput)==0)
  {
    if((millis()-caspitTime)>150){
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


void Drive(){
  caspitCapot();
  if(!caspitState()){
    if((leftSensorOut < black) || (rightSensorOut < black)){
      if(ifRightGreenSensor || ifLeftGreenSensor)
        turnOnGreen();

      else
        checkForward();
    }

    else
      drive1(60,60);

  }
  else
    DaniBoost(200,200);  
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


void checkForward(){
  while(M1Spin < 1 && M2Spin < 1){
    motors(60,60);
    updateSensors();
  }

  if(midSensor < black)
    return;
  else{
    while(M1Spin 0 && M2Spin 0){
      motors(60,60);
      updateSensors();
    }
    turnAround();  
  }
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

void turnOnGreen(){
  if(ifLeftGreenSensor){
    motors(60,-60);
    delay(50);
    while(rightSensorMid > black){
      updateSensors();
    }
  }

  else if(ifRightGreenSensor){
    motors(60,-60);
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
    
 // pinMode(trigPin,OUTPUT);   
  //trigUs();
  caspit_init();
  servo_init();
  colorSensor_init();
  
  AL.attach(arm_left);
  AR.attach(arm_right);
  GL.attach(grip_left);
  GR.attach(grip_right);
}

void loop(){
  updateSensors();
  Drive();

}


