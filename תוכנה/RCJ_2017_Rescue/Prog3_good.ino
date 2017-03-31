#define DEBUG1 0  // motor speed left/right
#define DEBUG2 0 // sensors digital values
#define DEBUG3 0 // sensors analog values
#define DEBUG4 0 // sensors analog values RAW
#define DEBUG5 1 // UltraSonic Sensor
#define DEBUG6 0 // Mercury sensor



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

// Mercury Sensor
#define caspitInput 2

volatile boolean cFlag=false;
unsigned long caspitTime;
unsigned long time = 0;
unsigned long duration;
int distance = 0;
int rightSensorMidRAW,leftSensorMidRAW,rightSensorOutRAW,leftSensorOutRAW;
int rightSensorMid,leftSensorMid,rightSensorOut,leftSensorOut;
int rightSub,leftSub;
int state = 0;

boolean digitalSensor[4];
int analogSensor[4];
String sensorNum[4] ={"rightSensorOut(A2)" , "rightSensorMid(A0)" , "leftSensorMid(A3)" , "leftSensorOut(A1)"};


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
  detachInterrupt(5);  
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
    }
    
void updateDigital() {
  for(int i = 0; i++; i< 4){
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
    for(int i = 0; i++; i<4){
        Serial.print(sensorNum[i] );
        Serial.print(" = ");
        Serial.println(digitalSensor[i] );
      }
    }
 
 if(DEBUG3){
    for(int i = 0; i++; i<4){
        Serial.print(sensorNum[i] );
        Serial.print(" = ");
        Serial.println(digitalSensor[i]);
    }
 }   
 
if(DEBUG4){
   Serial.print(" rightSensorOut (A2) : ");
    Serial.print(rightSensorOutRAW);
    Serial.print(" rightSensorMid(A0): ");
    Serial.print(rightSensorMidRAW);
    Serial.print(" leftSensorMid(A3): ");
    Serial.print(leftSensorMidRAW);
    Serial.print(" leftSensorOut(A1): ");
    Serial.println(leftSensorOutRAW);
  }
  
  
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
  if(!caspitState()){
    if((leftSensorOut < black) || (rightSensorOut < black))
      turnAround(); 

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

  pinMode(caspitInput,INPUT_PULLUP);

  trigUs();

  attachInterrupt(0, caspit, CHANGE);
}

void loop(){
  updateSensors();
  Drive();
}
