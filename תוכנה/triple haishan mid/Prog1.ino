#define black 400 
#define rightMotor_A 6
#define rightMotor_B 7
#define rightMotor_PWM 8
#define leftMotor_A 4
#define leftMotor_B 5
#define leftMotor_PWM 9

int rightSensor,midSensor,leftSensor;
int rightSub,leftSub;

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
  rightSensor = analogRead(A0);
  leftSensor = analogRead(A1);
  midSensor = analogRead(A2);

  rightSensor = map(rightSensor, 0, 1023, -100, 100);
  leftSensor = map(leftSensor, 0, 1023, -100, 100);
  midSensor = map(midSensor, 0, 1023, -100, 100);

  leftSub = rightSensor - leftSensor;
  rightSub = leftSensor - rightSensor;

  motors(60+leftSub, 60+rightSub);
  
}




