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
#define DEBUG15 1

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
const int M2_ENC_A = 2;
const int M2_ENC_B = 3;
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
#define BakbookInput A12
boolean BakbookFlag = false;

// Ultrasonic Sensors
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
#define roomSwitchInput 12

boolean roomSwitchFlag = false;



//Infrared Line Sensors
#define rightSensorOutInput A0
#define rightSensorMidInput A1
#define leftSensorMidInput A2
#define leftSensorOutInput A3
#define midSensorInput A4
#define frontMidSensorInput A5
int rightSensorMidRAW,leftSensorMidRAW,midSensorRAW,rightSensorOutRAW,leftSensorOutRAW,frontMidSensorRAW;
int rightSensorMid,leftSensorMid,midSensor,rightSensorOut,leftSensorOut,frontMidSensor;
int rightSub,leftSub;

boolean digitalSensor[6];
int analogSensor[6];
int analogSensorRAW[6];
const char* sensorNum[] = {"rightSensorOut(A0) = " , " rightSensorMid(A1) = " , " leftSensorMid(A2) = " , " leftSensorOut(A3) = " , " midSensor(A4) = "," frontMidSensor(A5) = "};




//Servos Functions
void moveUpFast(){
  AR.write(140, 100, false);
  AL.write(40, 100,false);
}

void moveUp(){
  AR.write(155, 30, false);
  AL.write(25, 30,false);
  
}

void moveHalfUp(){
  AR.write(125, 30, false);
  AL.write(55, 30,false);
}


void moveHalfUpWait(){
    AL.write(50, 50, true);
  AR.write(130, 50, true);
  
}

void moveHalfDown(){
  AL.write(130, 50, false);
  AR.write(60, 50, false);
  
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
void readSideIrCM(){
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
  } 
  
}

//Ultrasonic Sensors Functions
void trigUs(){
  digitalWrite(trigPin, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);
  //duration = pulseIn(echoPin,HIGH);
  //attachInterrupt(1, measurement, RISING);
}

void measurement(){
  //distance = duration/58;

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
  

  rightSensorOut = map(rightSensorOutRAW, 770, 80, 1000, 0);
  rightSensorMid = map(rightSensorMidRAW, 955 ,110, 1000, 0);
  leftSensorMid = map(leftSensorMidRAW, 985, 130,  1000, 0);
  leftSensorOut = map(leftSensorOutRAW, 830, 85,1000, 0);
  midSensor = map(midSensorRAW, 975 ,180, 1000, 0);
  frontMidSensor = map(frontMidSensorRAW, 130 ,40, 1000, 0);


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
  readSideIrCM();
  
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
          turnAround();
     //   checkForward();
    }
  

    else{
      drive1(60,60);
      Bakbook();
    }
  }
  else if(caspitState())
    DaniBoost(150,150);
  
  else if(caspitDownState()){
    DaniBoost(60,60);
    
  }
} 

void drive1(int speed1,int speed2){

  int rightSensorMid1 = map(rightSensorMid, 0, 1000, -80, 80);
  int leftSensorMid1 = map(leftSensorMid, 0, 1000, -80, 80);
  int rightSensorOut1 = map(rightSensorOut, 0, 1000, -80, 80);
  int leftSensorOut1 = map(leftSensorOut, 0, 1000, -80, 80);

  int error = (rightSensorMid1 - leftSensorMid1);

  motors(speed1 + error,speed2 - error);
}

void boomRoom(){ 
  
/*  double error = 4.0 - irDistance;
  error = map(error, 0 , 10, -70,70);
  motors(70+ error ,70 - error);
  */
  
  double error = (sideDistance - 3.5 )*20;
  motors(70+error,70 - error*0.2);
 
    
}

void DaniBoost(int speed3,int speed4){

  int rightSensorMid1 = map(rightSensorMid, 0, 1000, 0, 60);
  int leftSensorMid1 = map(leftSensorMid, 0, 1000, 0, 60);

  int error = (rightSensorMid1 - leftSensorMid1);

  motors(speed3 + error,speed4 - error); 
}


void checkGreen(){
    stopingz();
    delay(10);
    motors(70,70);
    delay(50);
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
 delay(10);

  if(frontMidSensor < black){
    driveForCMm1(2);
    return;
   }
  else 
    turnAround();
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


//Room Functions
#define leftTriangleSwitchInput 10
#define rightTriangleSwitchInput 11
#define wallSwitchInput 12

double x,y;
volatile boolean ifWallSwitch,ifTriangleSwitch;
volatile boolean wallFlag,triangleFlagLeft,triangleFlagRight;



void room_init(){
   pinMode(leftTriangleSwitchInput, INPUT_PULLUP);
   pinMode(rightTriangleSwitchInput, INPUT_PULLUP);
   pinMode(wallSwitchInput, INPUT_PULLUP);
   
   
  attachPinChangeInterrupt(leftTriangleSwitchInput, setTriangleFlagLeft , RISING);
  attachPinChangeInterrupt(rightTriangleSwitchInput, setTriangleFlagRight , RISING);
  attachPinChangeInterrupt(wallSwitchInput, setWallFlag , RISING);
  
  moveUp();
  
}

void setTriangleFlagLeft(){
  triangleFlagLeft = true;
}

void setTriangleFlagRight(){
  triangleFlagRight = true;
}

void setWallFlag(){
  wallFlag = true;
}

void roomProcess(){
  if(sideDistance < 12)
    boomRoom();
  
  else
    motors(70,70);
  
  if(wallFlag || triangleFlagLeft || triangleFlagRight){
    if(wallFlag){
      driveForCMm1(-2);
      pointTurn(90,true);    
    }
        wallFlag = false;
    
  }
  
  if(triangleFlagLeft || triangleFlagRight){

    if(triangleFlagLeft){
      motors(120,0);
      while(digitalRead(rightTriangleSwitchInput) == 1);
      
    }
    else if(triangleFlagRight){
      motors(0,120);
      while(digitalRead(leftTriangleSwitchInput) == 1);
        
    }
    
    triangleFlagLeft = false;
    triangleFlagRight = false;
   
    stopingz();
    delay(50);
    driveForCMm1(-1);
    pointTurn(90,true);
   motors(70,70);
    while(sideDistance > 10){

        updateSensors();
    }
     driveForCMm1(2);
     
  }
  
  
 
  
}

boolean roomFlag = true;

void setup(){
//Serial.begin(115200);
  AL.attach(arm_left);
  AR.attach(arm_right);
  GL.attach(grip_left);
  GR.attach(grip_right);
  
  for(int i = 4; i < 8; i ++)
    pinMode(i,OUTPUT);
    
    pinMode(rightGreenSensor,INPUT);
    pinMode(leftGreenSensor,INPUT);
    pinMode(roomSwitchInput, INPUT_PULLUP);
   
    attachPinChangeInterrupt(roomSwitchInput, setRoomFlag, RISING);
    
   pinMode(trigPin,OUTPUT);   
  trigUs();
  caspit_init();
  servo_init(); 
  colorSensor_init();
  Bakbook_init();
  
    

}


void loop(){
  updateSensors();

 /* if(!roomSwitchFlag){
   //updateSensors();
   //Drive();
 
  }
*/  
  //else if(roomSwitchFlag){
    if(roomFlag){
    room_init();
    roomFlag = false;
    }
    roomProcess();

//boomRoom();

/*   stopingz();
    while(1);
  }*/

}

