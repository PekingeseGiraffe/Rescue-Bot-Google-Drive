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
#define DEBUG11 0 // Bakbook Switch
#define DEBUG12 0 // Green color Flag
#define DEBUG13 0 // Front sensors analog values
#define DEBUG14 0 // Front sensors analog values RAW
#define DEBUG15 0 // Side Ir Distance Sensor

// Black Line Value
#define black 250 

//encoders
#define CPR 2248.86
const double robotDiameter = 18;
const double wheelDiameter =  8.2;
const double wheelCircumference = wheelDiameter*PI;
const double  CPRcm  = CPR/wheelCircumference;

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

// Bakbook Switch
#define BakbookInput 12
boolean BakbookFlag = false;

// Ultrasonic Sensor
#define trigPin 14
#define echoPin 3
unsigned long time = 0;
unsigned long duration;
double distance = 0;

//Infrared Distance Sensor
#define sideIrInput A10
double sideVoltage,sideDistance;

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
volatile boolean leftGreenFlag = false;
volatile boolean rightGreenFlag = false;
volatile unsigned long greenTimeout;

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
#define frontMidSensorInput A5
#define frontLeftSensorInput A6
#define frontRightSensorInput A7
int rightSensorMidRAW,leftSensorMidRAW,midSensorRAW,rightSensorOutRAW,leftSensorOutRAW,frontMidSensorRAW,frontLeftSensorRAW,frontRightSensorRAW;
int rightSensorMid,leftSensorMid,midSensor,rightSensorOut,leftSensorOut,frontMidSensor,frontLeftSensor,frontRightSensor;
int rightSub,leftSub;

volatile boolean leftOutFlag = false;
volatile boolean rightOutFlag = false;



boolean digitalSensor[6];
int analogSensor[6];
int analogSensorRAW[6];
int frontAnalogSensor[3];
int frontAnalogSensorRAW[3];
const char* sensorNum[] = {"rightSensorOut(A0) = " , " rightSensorMid(A1) = " , " leftSensorMid(A2) = " , " leftSensorOut(A3) = " , " midSensor(A4) = "," frontMidSensor(A5) = "};
const char* frontSensorNum[] = {"frontRightSensor(A7) = " , " frontMidSensor(A5) = " , " frontLeftSensor(A6) = " };




//Servos Functions
void moveUpFast(){
  AR.write(140, 100, false);
  AL.write(40, 100,false);
}

void moveUp(){
  AR.write(140, 30, false);
  AL.write(40, 30,false);
  
}

void moveHalfUp(){
  AR.write(130, 30, false);
  AL.write(50, 30,false);
}


void moveHalfUpWait(){
    AL.write(50, 50, true);
  AR.write(130, 50, true);
  
}

void moveHalfDown(){
  AL.write(105, 50, false);
  AR.write(85, 50, false);
  
}


void moveDown(){
  AL.write(115, 50, false);
  AR.write(75, 50, false);
}

void openGrip(){
  GR.write(30);
  GL.write(130);
  
}

void openHalfGripWait(){
    GR.write(60,100, true);
  GL.write(100,100, true);
  
}



void openGripWait(){
  GR.write(30, 50, true);
  GL.write(130,50, true);
  
}
void closeGrip(){
  GL.write(65,50,false);
  GR.write(85,50,false);
}

void servo_init(){
  openHalfGripWait();
  moveHalfUp();
}



// Bakbook Function
void Bakbook_init(){
  pinMode(BakbookInput, INPUT_PULLUP);
  attachPinChangeInterrupt(BakbookInput, turnBakbookFlag, RISING);
}

void turnBakbookFlag(){
  BakbookFlag = true;
}

//Infrared Distance Function
double getSideCM(){
  sideVoltage = analogRead(sideIrInput)/204.6;
  if (sideVoltage > 0.4) //y = 21x + 0.19
  { 
     sideDistance = (sideVoltage-0.19)/21;
     sideDistance = 1/sideDistance;
     sideDistance -= 6;
     //Serial.print(y);
     //Serial.print(' ');
     if(DEBUG15){
     Serial.print("Side Distance = ");
     Serial.println(sideDistance);
     }

     return sideDistance;
  } 
  
}

//Ultrasonic Sensors Functions
void trigUs(){
  digitalWrite(trigPin, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);

  attachInterrupt(1, startCount, RISING);
}

void startCount(){
  duration = micros();
  detachInterrupt(1);
  attachInterrupt(1, measurement, FALLING);
}

void measurement(){
  duration = micros() - duration;
  distance = duration/58;

  if(DEBUG5){
    Serial.print("Distance = ");
    Serial.println(distance);
  }
  detachInterrupt(1);  
}

//Green Sensors Functions
void colorSensor_init(){
  attachPinChangeInterrupt(rightGreenSensor, greenColorSensorState1, RISING);
  attachPinChangeInterrupt(leftGreenSensor, greenColorSensorState2, RISING);
}


void colorSensor_detach(){
  detachPinChangeInterrupt(rightGreenSensor);
  detachPinChangeInterrupt(leftGreenSensor);
}

void greenColorSensorState1(){
  rightGreenFlag = true;
 // if(digitalRead(leftGreenSensor))
   // leftGreenFlag = true;

  greenTimeout = millis();

 if(DEBUG10){
    Serial.print("RightGreenSensor : ");
    Serial.print(ifRightGreenSensor);
    Serial.print(" LeftGreenSensor : ");
    Serial.println(ifLeftGreenSensor);
 }
}

void greenColorSensorState2(){
  leftGreenFlag = true;
  // if(digitalRead(rightGreenSensor))
  // rightGreenFlag = true;
   
   greenTimeout = millis(); 
}

void clearGreenFlag(){
  rightGreenFlag = false;
  leftGreenFlag = false;
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
  frontMidSensorRAW = analogRead(frontMidSensorInput);
  frontLeftSensorRAW = analogRead(frontLeftSensorInput);
  frontRightSensorRAW = analogRead(frontRightSensorInput);
  

  rightSensorOut = map(rightSensorOutRAW, 860, 80, 1000, 0);
  rightSensorMid = map(rightSensorMidRAW, 985 ,110, 1000, 0);
  leftSensorMid = map(leftSensorMidRAW, 985, 125,  1000, 0);
  leftSensorOut = map(leftSensorOutRAW, 915, 85,1000, 0);
  midSensor = map(midSensorRAW, 990 ,170, 1000, 0);
  frontMidSensor = map(frontMidSensorRAW, 690 ,260 , 1000, 0);
  frontRightSensor  = map(frontRightSensorRAW, 525 ,140 , 1000, 0);
  frontLeftSensor = map(frontLeftSensorRAW, 540 ,150 , 1000, 0);


  if(leftSensorOut < black)
    leftOutFlag = true;

  if(rightSensorOut < black)
    rightOutFlag = true;  

  analogSensor[0] = rightSensorOut;
  analogSensor[1] = rightSensorMid;
  analogSensor[2] = leftSensorMid;
  analogSensor[3] = leftSensorOut;
  analogSensor[4] = midSensor;
  analogSensor[5] = frontMidSensor;

  analogSensorRAW[0] = rightSensorOutRAW;
  analogSensorRAW[1] = rightSensorMidRAW;
  analogSensorRAW[2] = leftSensorMidRAW;
  analogSensorRAW[3] = leftSensorOutRAW;
  analogSensorRAW[4] = midSensorRAW;
  analogSensorRAW[5] = frontMidSensorRAW;
  
  frontAnalogSensor[0] = frontRightSensor;
  frontAnalogSensor[1] = frontMidSensor;
  frontAnalogSensor[2] = frontLeftSensor;
  
   frontAnalogSensorRAW[0] = frontRightSensorRAW;
  frontAnalogSensorRAW[1] = frontMidSensorRAW;
  frontAnalogSensorRAW[2] = frontLeftSensorRAW;
 }

void updateDigital() {
  for(int i = 0; i< 6; i++){
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
  
  
 if(millis() - greenTimeout > 700){
  clearGreenFlag();
 } 
  
if(millis() - time > 50){
    trigUs();
    time = millis();
  }

  if(DEBUG2){
    for(int c = 0; c < 6; c++)
    {
      for(int i = 0; i< 6; i++){
        Serial.print(sensorNum[i]);
        Serial.print(digitalSensor[i]);
      }
      Serial.println();
    }

  }

  if(DEBUG3){
    for(int c = 0; c < 6;c++){
      for(int j = 0; j< 6; j++){
        Serial.print(sensorNum[j]);
        Serial.print(analogSensor[j]);
      }
    Serial.println();
    }
  }


  if(DEBUG4){
    for( int c = 0; c < 6; c++)
    {
      for(int k = 0; k < 6; k++){
        Serial.print(sensorNum[k]);
        Serial.print(analogSensorRAW[k]);
      }
      Serial.println();
    }
  }
  
  
    if(DEBUG13){
    for( int c = 0; c < 3; c++)
    {
      for(int k = 0; k < 3; k++){
        Serial.print(frontSensorNum[k]);
        Serial.print(frontAnalogSensor[k]);
      }
      Serial.println();
    }
  }
  
   if(DEBUG14){
    for( int c = 0; c < 3; c++)
    {
      for(int k = 0; k < 3; k++){
        Serial.print(frontSensorNum[k]);
        Serial.print(frontAnalogSensorRAW[k]);
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
  
  if(DEBUG12){
     Serial.print(" RightGreenFlag = ");
    Serial.print(rightGreenFlag);
    Serial.print(" leftGreenFlag= ");
    Serial.println(leftGreenFlag);

  }

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
    moveHalfUp();
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
    if((leftSensorOut < black)  || (rightSensorOut < black)){
       stopingz();
     delay(50);
      if(rightGreenFlag || leftGreenFlag){
         checkGreen();
      }
      else
//     turnAround();
  checkForward();
    }
  

    else{
      drive1(70,70);
      Bakbook();
    }
  }
  else if(caspitState())
    DaniBoost(160,160);
  
  else if(caspitDownState()){
    DaniBoost(60,60);
    
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


void checkGreen(){

    if(rightGreenFlag && leftGreenFlag){
          Uspin();
            
        }
         else   
            turnOnGreen();
  
}

void Uspin(){
   colorSensor_detach();
   pointTurn(180,true);
   colorSensor_init();  
}

void turnOnGreen(){
  if(leftGreenFlag){
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
 }


void retardMode(){
  pointTurn(30,true);
  pointTurn(60,false);
  pointTurn(30,true);
  
  
}


void checkForward(){
 stopingz();
 delay(50);
 updateSensors();
 driveForCMm1(5);
 stopingz();

 delay(50);
  
  
updateSensors();
  if((midSensor < 150) || (rightSensorMid < 150 )|| (leftSensorMid < 150)){
    return;
   }
  else {
    driveForCMm1(-9);
    stopingz();
    delay(50);
    updateSensors();
    while((leftSensorOut < black)  || (rightSensorOut < black)){
          updateSensors();
          drive1(70,70);
    }
    turnAround();    
  }
}


void turnAround(){
  if(leftOutFlag){
    motors(70,-70);
    delay(200);
    while(rightSensorMid > black){
      readSensors();
    }
    leftOutFlag = false;
  }
  else if(rightOutFlag){
    motors(-70 ,70);
    delay(200);
    while(leftSensorMid > black){
      readSensors();   
    }
    rightOutFlag = false;
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
 double  M2temp=M2.read() /CPRcm;

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
  double ratio = (degree/360);
  double length = (robotDiameter*PI)/ wheelCircumference; 
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
  
  double ratio = (degree/360);
  double length = (robotDiameter*PI)/ wheelCircumference; 
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
  
   double ratio = (degree/360);
   double length = (2*(robotDiameter*PI))/ wheelCircumference;
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
   if(BakbookFlag)
  {
    driveForCMm1(-6);
    pointTurn(45, true);
    driveForCMm1(18);
    pointTurn(50, false);
    driveForCMm1(22);
    pointTurn(32, false);
    driveForCMm1(10);
    motors(70,70);
    updateSensors();
    while(rightSensorMid > black)
      updateSensors();
      
    motors(-70,70);
    while(leftSensorMid > black)
      updateSensors();

    
  }
  BakbookFlag = false;
}

void setup(){
Serial.begin(115200);
  AL.attach(arm_left);
  AR.attach(arm_right);
  GL.attach(grip_left);
  GR.attach(grip_right);
  
  for(int i = 4; i < 8; i ++)
    pinMode(i,OUTPUT);
    
    pinMode(rightGreenSensor,INPUT);
    pinMode(leftGreenSensor,INPUT);
    pinMode(roomSwitchInput, INPUT_PULLUP);
   
    attachInterrupt(roomSwitchInt, setRoomFlag, RISING);
    
    pinMode(trigPin,OUTPUT);   
  trigUs();
  caspit_init();
  servo_init(); 
  colorSensor_init();
  Bakbook_init();

}


void loop(){
  //updateSensors();

  //if(!roomSwitchFlag){
   updateSensors();
  // Drive();
   Serial.println(getSideCM());
 /*
  }
  
  else if(roomSwitchFlag){
    stopingz();
    while(1);
  }
*/
}

