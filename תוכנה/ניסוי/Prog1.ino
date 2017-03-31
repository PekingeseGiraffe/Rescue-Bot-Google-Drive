#define black 400 
#define rightMotor_A 6
#define rightMotor_B 7
#define rightMotor_PWM 8
#define leftMotor_A 4
#define leftMotor_B 5
#define leftMotor_PWM 9

int rightSensorMid,leftSensorMid,rightSensorOut,leftSensorOut;
int rightSub,leftSub;

void stopingz(){
  motors(0,0);
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
  
}

void loop(){
  rightSensorMid = analogRead(A0);
  leftSensorMid = analogRead(A3);
  rightSensorOut = analogRead(A2);
  leftSensorOut=analogRead(A1);

rightSensorMid = map(rightSensorMid,520 , 70, 1000, 0);
 leftSensorMid = map(leftSensorMid, 880, 100,  1000, 0);
  rightSensorOut = map(rightSensorOut, 570, 60, 1000, 0);
  leftSensorOut = map(leftSensorOut, 465, 65,1000, 0);

  rightSensorMid = map(rightSensorMid, 0, 1023, -100, 100);
  leftSensorMid = map(leftSensorMid, 0, 1023, -100, 100);
  rightSensorOut = map(rightSensorOut, 0, 1023, -100, 100);
  leftSensorOut = map(leftSensorOut, 0, 1023, -100, 100);

  rightSub = rightSensorMid - leftSensorMid;
  leftSub = leftSensorMid - rightSensorMid;

  motors(50+leftSub, 50+rightSub);
  
}




