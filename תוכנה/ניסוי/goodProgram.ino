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
  attachInterrupt(5, startCount, RISING);
}

void startCount(){
  time = micros();
  detachInterrupt(5);
  attachInterrupt(5, measurement, FALLING)
}

void measurement(){
  time = micros() - time;
  distance = time/58;
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

  rightSensorMid = map(rightSensorMid,575 ,80, 1000, 0);
  leftSensorMid = map(leftSensorMid, 952, 100,  1000, 0);
  rightSensorOut = map(rightSensorOut, 635, 65, 1000, 0);
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
    
void drive1(int speed1,int speed2){
  
  int rightSensorMid1 = map(rightSensorMid, 0, 1023, -100, 100);
  int leftSensorMid1 = map(leftSensorMid, 0, 1023, -100, 100);
  int rightSensorOut1 = map(rightSensorOut, 0, 1023, -100, 100);
  int leftSensorOut1 = map(leftSensorOut, 0, 1023, -100, 100);
  rightSub = rightSensorMid1 - leftSensorMid1;
  leftSub = leftSensorMid1 - rightSensorMid1;
   motors(speed1 + rightSub,speed2 + leftSub);
}

void drive2(int speed1,int speed2){
  
  int rightSensorMid1 = map(rightSensorMid, 0, 1023, 10, 100);
  int leftSensorMid1 = map(leftSensorMid, 0, 1023, 10, 100);
  rightSub = rightSensorMid1 - leftSensorMid1;
  leftSub = leftSensorMid1 - rightSensorMid1;
   motors(speed1 + rightSub,speed2 + leftSub);
}


void turnAround(){
    if(leftSensorOut < black){
      motors(60,-60);
      delay(50);
        while(rightSensorMid > black){
              readSensors();
              Serial.println(1);
        }
      }
     else if(rightSensorOut < black){
       motors(-60 ,60);
       delay(50);
        while(leftSensorMid > black){
            readSensors();    
             Serial.println(2);
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

  pinMode(caspitInput,INPUT);

  trigUs();

  attachInterrupt(0, cFlag, CHANGE);
}

void loop(){
  readSensors();

  if(!ifAngle){
    if((leftSensorOut < black) || (rightSensorOut < black))
      state = 1;
    else
      state = 0;    

    switch (state){
      case 0:
       drive1(80,80);
       break;

      case 1:
        turnAround();
        break;

      default:
       drive1(80,80);
       break;

    }
  else
    drive2(150,150);  
}