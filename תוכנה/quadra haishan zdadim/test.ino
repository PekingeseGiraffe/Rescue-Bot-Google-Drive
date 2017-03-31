#define black 250 
#define rightMotor_A 6
#define rightMotor_B 7
#define rightMotor_PWM 8
#define leftMotor_A 4
#define leftMotor_B 5
#define leftMotor_PWM 9
#define trigPin 10
#define echoPin 2
#define caspitInput 3

unsigned long timeUltra = 0;
unsigned long duration;
int distanceUltra = 0;
int rightSensorMid,leftSensorMid,rightSensorOut,leftSensorOut;
int rightSub,leftSub;
int caspitCounter = 0;





void trigUs(){
  digitalWrite(trigPin, 1);
  delayMicroseconds(10);
  digitalWrite(trigPin,0);
  attachInterrupt(0, measurement, RISING);
}


volatile boolean cFlag=false;



void caspit(){
  cFlag = true;
}
  

int distance;
void measurement(){
  timeUltra = pulseIn(echoPin, HIGH);
  distanceUltra = timeUltra/58;
  Serial.print("Distance = ");
  Serial.println(distance);
  detachInterrupt(0);  
}

void stopingz(){
  motors(0,0);
}


unsigned long tTime;

boolean caspitState(){
  if(cFlag){
    if(digitalRead(3)==0){
    tTime=millis();
    }
  }
   else if(digitalRead(3)==0)
    {
        if((millis()-tTime)>150)
        return true;
    }
    cFlag=false;
    return false;
  }


void readSensors(){
 
 
  rightSensorMid = analogRead(A0);
  leftSensorMid = analogRead(A3);
  rightSensorOut = analogRead(A2);
  leftSensorOut=analogRead(A1);

  rightSensorMid = map(rightSensorMid,600 ,80, 1000, 0);
  leftSensorMid = map(leftSensorMid, 952, 100,  1000, 0);
  rightSensorOut = map(rightSensorOut, 600, 65, 1000, 0);
  leftSensorOut = map(leftSensorOut, 510, 65,1000, 0);
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

int Speed =80;

unsigned long tmpTime;
boolean cOn=false;
void drive(){
//Speed =150;
  if(caspitState()){
tmpTime=millis();
drive2();
cOn=true;
}
else if(cOn){
if(millis()-tmpTime>400)
drive2();
cOn=false;
}

else {
  Speed=60;
  if(leftSensorOut<black || rightSensorOut<black)
  turnAround();
 else{
  int rightSensorMid1 = map(rightSensorMid, 0, 1023, -100, 100);
  int leftSensorMid1 = map(leftSensorMid, 0, 1023,   -100, 100);
  int rightSensorOut1 = map(rightSensorOut, 0, 1023,   -100, 100);
  int leftSensorOut1 = map(leftSensorOut, 0, 1023,   -100, 100);
  rightSub = rightSensorMid1- leftSensorMid1;
  leftSub = leftSensorMid1 - rightSensorMid1;
  motors(Speed + rightSub,Speed + leftSub);
   /*Serial.print(" RIGHT: ");
   Serial.print(Speed + rightSub);
   Serial.print(" left: ");
   Serial.println(Speed + leftSub);*/

 }
 
}
  
}

void drive2(){
 Speed = 120;
  int rightSensorMid1 = map(rightSensorMid, 0, 1023, 0, 50);
  int leftSensorMid1 = map(leftSensorMid, 0, 1023, 0, 50);
  int rightSensorOut1 = map(rightSensorOut, 0, 1023, 0, 50);
  int leftSensorOut1 = map(leftSensorOut, 0, 1023, 0, 50);
  rightSub = rightSensorMid1- leftSensorMid1;
  leftSub = leftSensorMid1 - rightSensorMid1;
  motors(Speed + rightSub,Speed + leftSub);
}

void turnAround(){
  if(leftSensorOut < black){
    motors(60,-60);
    delay(100);
    while(rightSensorMid > black){
      readSensors();
      Serial.println(1);
    }
    }
  else if(rightSensorOut < black){
    motors(-60 ,60);
    delay(100);
    while(leftSensorMid > black){
      readSensors();    
      Serial.println(2);
    }
  }

}

void motors(int left, int right){
  if(left>255)
  left=255;
  if(left <-255)
  left=-255;
   if(right>255)
  right=255;
  if(right <-255)
  right=-255;
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

  pinMode(caspitInput,INPUT);
  attachInterrupt(1, caspit, CHANGE); 
  trigUs();
}

void loop(){
  
 motors(60,-60);
 delay(500);
/* stopingz();
  motors(60,60);
  delay(800);
  stopingz();
  motors(20,70);
  delay(1500);
   motors(50,70);
  delay(1000);
  stopingz();*/
while(1);
  
  

}

