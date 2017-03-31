
//#include <Servo.h> 
 
//Servo kl,kr;
//Servo zl,zr;  
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
#define trigPin 10
#define echoPin 18

// Mercury Sensor
#define caspitInput 2

volatile boolean cFlag=false;
unsigned long caspitTime;
unsigned long time = 0;
unsigned long duration;
int distance = 0;
int rightSensorMid,leftSensorMid,rightSensorOut,leftSensorOut;
int rightSub,leftSub;
int state = 0;

void trigUs(){
  digitalWrite(trigPin, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);
  duration = pulseIn(echoPin,HIGH);
  attachInterrupt(5, measurement, RISING);
}

void measurement(){
  distance = duration/58;
  Serial.print("Distance = ");
  Serial.println(distance);
  detachInterrupt(5);  
}

void stopingz(){
  motors(0,0);
}

void readSensors(){
  rightSensorMid = analogRead(A0);
  leftSensorMid = analogRead(A3);
  rightSensorOut = analogRead(A2);
  leftSensorOut=analogRead(A1);

   /*
 rightSensorOut = map(rightSensorOut, 980, 160, 1000, 0);
 rightSensorMid = map(rightSensorMid,980 ,165, 1000, 0);
leftSensorMid = map(leftSensorMid, 985, 170,  1000, 0);
leftSensorOut = map(leftSensorOut, 925, 160,1000, 0);
  */ 
  
  /*
    Serial.print(" leftSensorOut(A1): ");
    Serial.print(leftSensorOut);
    Serial.print(" leftSensorMid(A3): ");
    Serial.print(leftSensorMid);
    Serial.print(" rightSensorMid(A0): ");
    Serial.print(rightSensorMid);
    Serial.print(" rightSensorOut(A2): ");
    Serial.println(rightSensorOut);*/
    }

void caspit(){
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
        if((millis()-caspitTime)>150)
        return true;
    }
    cFlag=false;
    return false;
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
              readSensors();
        }
      }
     else if(rightSensorOut < black){
       motors(-60 ,60);
       delay(50);
        while(leftSensorMid > black){
            readSensors();   
        }
     }
}

void motors(int left, int right){
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
 //kr.attach(43); 
  //kl.attach(45); 
 // zl.attach(44); 
 // zr.attach(42); 
  pinMode(caspitInput,INPUT_PULLUP);


  trigUs();
//closeZ();
//moveUp();
  attachInterrupt(0, caspit, CHANGE);
}


/* 
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
 }*/


void loop(){
  readSensors();

  if(millis() - time > 50){
    trigUs();
    time = millis();
  }

  if(!caspitState()){
    if((leftSensorOut < black) || (rightSensorOut < black))
      state = 1;
    else
      state = 0;    
//moveDown ();
    switch (state){
      case 0:
       drive1(60,60);
       break;

      case 1:
        turnAround();
        break;

      default:
       drive1(60,60);
       break;

    }
  }
  else{
//  moveUp();
    DaniBoost(200,200);  
  }
}
