//#include <Servo.h>
#include <Encoder.h>
#include <PinChangeInt.h>
#include <VarSpeedServo.h>

#define DEBUG1 0 // motor speed left/right
#define DEBUG2 0 // sensors digital values
#define DEBUG3 0 // sensors analog values
#define DEBUG4 0 // sensors analog values RAW
#define DEBUG5 0 // UltraSonic Sensor
#define DEBUG6 0 // Mercury sensor
#define DEBUG7 0 // Mercury 'UP' State
#define DEBUG8 0 // Mercury 'DOWN' State
#define DEBUG9 0 // Encoders Spins
#define DEBUG10 0 // Green color sensor digital
#define DEBUG11 1 // Ultrasonic Bakbook Sensor

// Black Line Value
#define black 250 

//encoders
#define CPR 2248.86
#define CPRcm 102.26205
#define wheelPerimeter 22
const double  fullRotation = 360;
const double robotDiameter = 25.5;


const int M1_ENC_A = 18;
const int M1_ENC_B = 19;
const int M2_ENC_A = 20;
const int M2_ENC_B = 21;
Encoder M1(M1_ENC_A, M1_ENC_B);
Encoder M2(M2_ENC_A, M2_ENC_B);
double M1Spin, M2Spin,M1SpinCM,M2SpinCM;


//servos
const int arm_left =  44, arm_right =  42;
const int grip_left = 45, grip_right = 43;
VarSpeedServo AL,GL,AR,GR; //Arm left, Grip left, Arm right, Grip right

// Engine Pin Layout
#define rightMotor_A 6
#define rightMotor_B 7
#define rightMotor_PWM 8
#define leftMotor_A 4
#define leftMotor_B 5
#define leftMotor_PWM 9

// Ultrasonic Sensors
#define trigPinBakbook 15
#define echoPinBakbook A13
unsigned long timeBakbook = 0;
unsigned long durationBakbook;
double distanceBakbook = 0;


#define trigPin 14
#define echoPin 3
unsigned long time = 0;
unsigned long duration;
double distance = 0;

// Mercury Sensor
#define caspitInclineInput A8
#define caspitDeclineInput A9
volatile boolean cInclineFlag=false;
volatile boolean cDeclineFlag=false;
unsigned long caspitTime;
unsigned long caspitInclineTime;
unsigned long caspitDeclineTime;

// Green Color Sensors
#define rightGreenSensor A14
#define leftGreenSensor A15
boolean ifRightGreenSensor, ifLeftGreenSensor;
volatile boolean leftGreenFlag = false, rightGreenFlag = false;
unsigned long greenTimeout;

//Room switch
#define roomSwitchInput 2
#define roomSwitchInt  0
boolean roomSwitchFlag = false;

//Infrared Line Sensors
#define rightSensorOutInput A0
#define rightSensorMidInput A1
#define leftSensorMidInput A2
#define leftSensorOutInput A3
#define midSensorInput A4
int rightSensorMidRAW,leftSensorMidRAW,midSensorRAW,rightSensorOutRAW,leftSensorOutRAW;
int rightSensorMid,leftSensorMid,midSensor,rightSensorOut,leftSensorOut;
int rightSub,leftSub;

boolean digitalSensor[5];
int analogSensor[5];
int analogSensorRAW[5];
const char* sensorNum[] = {"rightSensorOut(A0) = " , " rightSensorMid(A1) = " , " leftSensorMid(A2) = " , " leftSensorOut(A3) = " , " midSensor(A4) = "};




//Servos Functions
void moveUpFast(){
  AR.write(140, 100, false);
  AL.write(40, 100,false);
}

void moveUp(){
  AR.write(140, 30, false);
  AL.write(40, 30,false);
  
}

void moveHalfDown(){
  AL.write(125, 50, false);
  AR.write(65, 50, false);
  
}


void moveDown(){
  AL.write(115, 50, false);
  AR.write(75, 50, false);
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
  moveUpFast();
//  closeGrip();
}


//Ultrasonic Sensors Functions
/*void trigUsBakbook(){
  digitalWrite(trigPinBakbook, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);
  durationBakbook = pulseIn(echoPinBakbook,HIGH);
  attachPinChangeInterrupt(echoPinBakbook, measurementBakbook, RISING);
}

void measurementBakbook(){
  distanceBakbook = durationBakbook/58;

  if(DEBUG11){
    Serial.print("DistanceBakbook = ");
    Serial.println(distanceBakbook);
  }
  detachPinChangeInterrupt(echoPinBakbook);  
}*/


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

//Room Functions
void setRoomFlag(){
  roomSwitchFlag = true;
}


void stopingz(){
  motors(0,0);
}


// Infrared Sensors Functions
void readSensors(){
  rightSensorMidRAW = analogRead(rightSensorMidInput);
  leftSensorMidRAW = analogRead(leftSensorMidInput);
  midSensorRAW = analogRead(midSensorInput);
  rightSensorOutRAW = analogRead(rightSensorOutInput);
  leftSensorOutRAW =analogRead(leftSensorOutInput);

  rightSensorOut = map(rightSensorOutRAW, 1000, 105, 1000, 0);
  rightSensorMid = map(rightSensorMidRAW, 1000 ,135, 1000, 0);
  leftSensorMid = map(leftSensorMidRAW, 1000, 160,  1000, 0);
  leftSensorOut = map(leftSensorOutRAW, 1000, 125,1000, 0);
  midSensor = map(midSensorRAW, 1000 ,190, 1000, 0);

  analogSensor[0] = rightSensorOut;
  analogSensor[1] = rightSensorMid;
  analogSensor[2] = leftSensorMid;
  analogSensor[3] = leftSensorOut;
  analogSensor[4] = midSensor;

  analogSensorRAW[0] = rightSensorOutRAW;
  analogSensorRAW[1] = rightSensorMidRAW;
  analogSensorRAW[2] = leftSensorMidRAW;
  analogSensorRAW[3] = leftSensorOutRAW;
  analogSensorRAW[4] = midSensorRAW;
}

void updateDigital() {
  for(int i = 0; i< 5; i++){
    if(analogSensor[i] < black)
      digitalSensor[i] = true;
    else
      digitalSensor[i] = false;   
  }
}  

//Update Sensors
void updateSensors(){
  readSensors();
  updateDigital();
  caspitState();
  caspitUpState();
  caspitDownState();
  MotorSpins();
  MotorSpinsCM();
  
  
 if(millis() - greenTimeout > 1000){
  clearGreenFlag();
 } 
  
if(millis() - time > 50){
    trigUs();
    time = millis();
  }
  
  /*
  if(millis() - timeBakbook > 50){
    trigUsBakbook();
    timeBakbook = millis();
  }*/


  if(DEBUG2){
    for(int c = 0; c < 5; c++)
    {
      for(int i = 0; i< 5; i++){
        Serial.print(sensorNum[i]);
        Serial.print(digitalSensor[i]);
      }
      Serial.println();
    }

  }

  if(DEBUG3){
    for(int c = 0; c < 5;c++){
      for(int j = 0; j< 5; j++){
        Serial.print(sensorNum[j]);
        Serial.print(analogSensor[j]);
      }
    Serial.println();
    }
  }


  if(DEBUG4){
    for( int c = 0; c < 5; c++)
    {
      for(int k = 0; k < 5; k++){
        Serial.print(sensorNum[k]);
        Serial.print(analogSensorRAW[k]);
      }
      Serial.println();
    }
  }

   if(DEBUG9){
    Serial.print(" M1Spin = ");
    Serial.print(M1Spin);
    Serial.print(" M2Spin =  ");
    Serial.println(M2Spin);

  }

}


//Green Sensors Functions
void greenColorSensorState(){
  
  ifRightGreenSensor = digitalRead(rightGreenSensor);
  ifLeftGreenSensor = digitalRead(leftGreenSensor);
  greenTimeout = millis();
  updateGreenFlag();
  if(DEBUG10){
    Serial.print("RightGreenSensor : ");
    Serial.print(ifRightGreenSensor);
    Serial.print(" LeftGreenSensor : ");
    Serial.println(ifLeftGreenSensor);
  }
}

void colorSensor_init(){
  attachPinChangeInterrupt(rightGreenSensor, greenColorSensorState, CHANGE);
  attachPinChangeInterrupt(leftGreenSensor, greenColorSensorState, CHANGE);
}


//Caspit Sensors Functions 
void caspit_init(){
  pinMode(caspitInclineInput, INPUT_PULLUP);
  pinMode(caspitDeclineInput, INPUT_PULLUP);
  attachPinChangeInterrupt(A8, setCaspitUpTime, CHANGE);
  attachPinChangeInterrupt(A9, setCaspitDownTime, CHANGE);
}

void setCaspitUpTime(){
   if(digitalRead(caspitInclineInput)==0)
      caspitTime=millis();
}    


void setCaspitDownTime(){
  if(digitalRead(caspitDeclineInput)==0)
      caspitDeclineTime=millis();
} 

boolean caspitState(){ 

  if(digitalRead(caspitInclineInput)==0){
    if((millis()-caspitTime)>150){
      if(DEBUG6)
        Serial.println("CaspitState : true");
      return true;
    }
  }

  if(DEBUG6)
    Serial.println("CaspitState = false");

  return false;
}

boolean caspitUpState(){
  
  if(digitalRead(caspitInclineInput)==0){
    if((millis()-caspitTime) > 200){
      if(DEBUG7)
        Serial.println("CaspitUpState : true");
      return true;  
    }
  }
      if(DEBUG7)
        Serial.println("CaspitUpState : false");

      return false;   
}


boolean caspitDownState(){
 
  if(digitalRead(caspitDeclineInput)==0){
    if((millis()-caspitDeclineTime) > 200){
      if(DEBUG8)
        Serial.println("CaspitDownState : true");
      return true;  
    }
  }
      if(DEBUG8)
        Serial.println("CaspitDownState : false");
   
      return false;   
}

void caspitCapot(){
  if(caspitUpState()){
    cInclineFlag = 1;
    moveHalfDown();
  }
    
  
  if(caspitDownState()){
    cDeclineFlag = 1;
    moveDown();  
}
   
  if((cDeclineFlag==1) && (cInclineFlag==1) && (digitalRead(caspitDeclineInput)==1) && (digitalRead(caspitInclineInput)==1) ){
    moveUp();
    cDeclineFlag = 0;
    cInclineFlag = 0;
    //dec =ירידה
    //inc =עליה
    }
}

//Drive Functions
void Drive(){
  caspitCapot();
  if(!caspitState()){
    if((leftSensorOut < black)  ||(rightSensorOut < black)){
      if(rightGreenFlag || leftGreenFlag){
        if(rightGreenFlag && leftGreenFlag)
            pointTurn(180,true);
         else   
            turnOnGreen();
      }
      else
        turnAround();
/*else
        checkForward();*/
    }
  

    else{
      drive1(70,70);
    }
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


void clearGreenFlag(){
  rightGreenFlag = false;
  leftGreenFlag = false;
  
}

void  updateGreenFlag(){
  if(ifRightGreenSensor)
      rightGreenFlag = true;
      
  if(ifLeftGreenSensor)
      leftGreenFlag = true;
  
}

unsigned long waitTimeGreen;

void turnOnGreen(){
  if(leftGreenFlag){
    motors(70,70);
    delay(100);
    
    
    
    pointTurn(45, false);
    delay(50);
          
          
          
          initDeg();
          motors(70,-70);
          
          
     
     while(rightSensorMid > black ){
    updateSensors();
       if( (moreTurn(45,false)))
       {
         
          motors(-70,70);
          while(leftSensorMid > black )
          updateSensors();
          break;
       }
    }
    

  }

  else if(rightGreenFlag){
     motors(70,70);
    delay(100);
     pointTurn(45, true);
    delay(50);
    
              initDeg();
          motors(-70,70);
          
          
     
     while(leftSensorMid > black ){
    updateSensors();
       if( (moreTurn(45,true)))
       {
         
          motors(70,-70);
          while(rightSensorMid > black )
          updateSensors();
          break;
       }
    }

    
  }
  clearGreenFlag();
  waitTimeGreen = millis();
 }


void checkForward(){
  driveForCMm2(1);

  if(midSensor < black)
    return;
  else{
    driveForCMm2(-1);
    turnAround();  
  }
}

void turnAround(){
  if(leftSensorOut < black){
    motors(70,-70);
    delay(50);
    while(rightSensorMid > black){
      updateSensors();
    }
  }
  else if(rightSensorOut < black){
    motors(-70 ,70);
    delay(50);
    while(leftSensorMid > black){
      updateSensors();   
    }
  }
}



void leftMotorDrive(int pwm){
  if (pwm > 0)
  {
    digitalWrite(leftMotor_A,1);
    digitalWrite(leftMotor_B,0);  
    analogWrite(leftMotor_PWM, pwm);
  }
  else
  {
    digitalWrite(leftMotor_A,0);
    digitalWrite(leftMotor_B,1);  
    analogWrite(leftMotor_PWM, -pwm);
  }
}

void rightMotorDrive(int pwm){
  if (pwm > 0)
  {
    digitalWrite(rightMotor_A,1);
    digitalWrite(rightMotor_B,0);  
    analogWrite(rightMotor_PWM, pwm);
  }
  else
  {
    digitalWrite(rightMotor_A,0);
    digitalWrite(rightMotor_B,1);  
    analogWrite(rightMotor_PWM, -pwm);
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

//Encoder Functions
void MotorSpins(){
  M1Spin = M1.read() /CPR;
  M2Spin = M2.read() /CPR;
}

void MotorSpinsCM(){
  M1SpinCM = M1.read() /CPRcm;
  M2SpinCM = M2.read() /CPRcm;
}


void driveForCMm1(double cm){
 double  M1temp=M1.read()/CPRcm;

 updateSensors();

  if(cm > 0){
    while (M1temp+cm>M1SpinCM){
      motors(70,70);
      updateSensors();
    }
  }

  else{
    while (M1temp+cm<M1SpinCM){
     motors(-70, -70);
      updateSensors();
    }
  }
}


void driveForCMm2(double cm){
 int  M2temp=M2.read() /CPRcm;

 updateSensors();

  if(cm > 0){
    while (M2temp+cm>M2SpinCM){
      motors(70,70);
      updateSensors();
    }
  }

  else{
    while (M2temp+cm<M2SpinCM){
     motors(-70, -70);
      updateSensors();
    }
  }
}

volatile double  M2temp2;
volatile double  M1temp2;
void initDeg(){
     M2temp2=M2.read() /CPR;
    M1temp2=M1.read() /CPR;
}

boolean moreTurn(double degree, boolean imClock){
 
   updateSensors();
  double ratio = (degree/fullRotation);
  double length = (robotDiameter*PI)/ wheelPerimeter; 
  double rotations = ratio * length;
  if(imClock){
    Serial.print(M2temp2+rotations);
    Serial.print(" ");
    Serial.println(M2Spin);
    if (M2temp2+rotations>M2Spin)
    //updateSensors();
    return false;
    else 
     return true;
  
  }
  else if(!imClock){
     if (M2temp2-rotations<M2Spin)
         return false;
         else
         return true;

  
}
return false;
}


void pointTurn(double degree, boolean imClock){
  double  M2temp=M2.read() /CPR;
  double  M1temp=M1.read() /CPR;
  
  double ratio = (degree/fullRotation);
  double length = (robotDiameter*PI)/ wheelPerimeter; 
  double rotations = ratio * length;
  if(imClock){
    while (M2temp+rotations>M2Spin){
    updateSensors();
      motors(-70,70);
  }
  }
  if(!imClock){
     while (M2temp-rotations<M2Spin){
        updateSensors();
       motors(70,-70);
  }
  }
  motors(0,0);
}

void pivotTurn(double degree, boolean right){
  double M2temp=M2.read() /CPR;
  double M1temp=M1.read() /CPR;
  
   double ratio = (degree/fullRotation);
   double length = (2*(robotDiameter*PI))/ wheelPerimeter;
   double rotations = ratio * length;
   
  if(right){
    while (M2temp+rotations>M2Spin){
    updateSensors();
    motors(0,70);
  }
  } 
  if(!right){
     while (M1temp+rotations>M1Spin){
      updateSensors();
       motors(70,0);
  }
  }
  motors(0,0);
}

void Bakbook(){
  if(distance < 10 )
  {
    pointTurn(50, true);
    driveForCMm1(17);
    pointTurn(30, false);
   driveForCMm1(9);
   pointTurn(30, false);
  driveForCMm1(17);
   
    
  }
  
}

void setup(){
Serial.begin(115200);
  for(int i = 4; i < 8; i ++)
    pinMode(i,OUTPUT);
    
    pinMode(rightGreenSensor,INPUT);
    pinMode(leftGreenSensor,INPUT);
    pinMode(roomSwitchInput, INPUT_PULLUP);
   
    attachInterrupt(roomSwitchInt, setRoomFlag, RISING);
    
  pinMode(trigPin,OUTPUT);   
//  pinMode(trigPinBakbook,OUTPUT);
 // trigUsBakbook();
  trigUs();
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

  if(!roomSwitchFlag){
  updateSensors();
 Drive();
 
  }
  
  else if(roomSwitchFlag){
    stopingz();
    while(1);
  }
  
}

