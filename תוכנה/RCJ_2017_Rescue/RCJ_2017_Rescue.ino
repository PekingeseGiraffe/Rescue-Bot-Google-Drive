#define DEBUG1 0  // motor speed left/right
#define DEBUG2 0 // sensors digital values
#define DEBUG3 0 // sensors analog values
#define DEBUG4 0 // sensors analog values RAW
#define DEBUG5 0 // last edge - left/right
#define DEBUG6 0 // line tracking 
#define DEBUG7 0 // US
#define DEBUG8 0 // Mercury sensor

#include <Servo.h>
#include <Encoder.h>


//encoders
const int M1_ENC_A = 18;
const int M1_ENC_B = 19;
const int M2_ENC_A = 20;
const int M2_ENC_B = 21;
Encoder M1(M1_ENC_A, M1_ENC_B);
Encoder M2(M2_ENC_A, M2_ENC_B);

//servos
const int arm_left = 43;
const int arm_right = 42;
const int grip_left = 45;
const int grip_right = 43;
Servo AL, GL, AR, GR; //arm left.. gripper right


//motors
const int EN1 = 8;
const int EN2 = 9;
const int M1A = 6;
const int M1B = 7;
const int M2A = 4;
const int M2B = 5;

//IR line senors
const int SEN1 = A1; // left sensor
const int SEN2 = A3;
const int SEN3 = A0;
const int SEN4 = A2; // right sensor

int s1, s2, s3, s4; // s1 - left,s2 - leftmid , s3 - right mid, s4 - right      - analog values
boolean S1 , S2 , S3 , S4; //WHITE - false , BLACK - true
const int BLK_VAL = 60;

const int SEN1_MAX = 970; //calibration values
const int SEN1_MIN = 125;
const int SEN2_MAX = 975;
const int SEN2_MIN = 190;
const int SEN3_MAX = 975;
const int SEN3_MIN = 150;
const int SEN4_MAX = 975;
const int SEN4_MIN = 140;

//US rangefinder
const int trigPin = 22;
const int echoPin = 3;
const int US_INT_NUM = 1;
int distance;
volatile unsigned long duration;

//Mercury sensor
const int merc_pin = 2;
const int merc_Int = 0;
const int merc_trigger_time = 150;
volatile boolean mFlag;
unsigned long merc_tmp;


int baseSpeed = 25; //base drive speed


void setup() {
  Serial.begin(250000);
  US_init();
  merc_init();
  //servo_init();
  motors_init();
  updateSensors();
  delay(50);

}

void servo_init() {
  const int pmin = 500, pmax = 2000;
  AL.attach(arm_left, pmin, pmax);
  AR.attach(arm_right, pmin, pmax);
  GL.attach(grip_left, pmin, pmax);
  GR.attach(grip_right, pmin, pmax);
  setArmAngle(40);
  grip(false);
}

void grip(boolean a) {
  if (a) { //grip
    GL.write(70);
    GR.write(110);
  }
  else { //release
    GL.write(110);
    GR.write(70);
  }
}

void setArmAngle(int deg) {
  AL.write(deg);
  AR.write(180 - deg);
  grip(false);
}

void merc_init() {
  pinMode(merc_pin, INPUT_PULLUP);
  attachInterrupt(merc_Int, setMercFlag, CHANGE);
}

void setMercFlag() {
  mFlag = true;
}

boolean MercSens() {
  if (mFlag) {
    if (digitalRead(merc_pin) == 0) {
      merc_tmp = millis();
    }
  }
  else if (digitalRead(merc_pin) == 0) {
    if ((millis() - merc_tmp) > merc_trigger_time) {
      if (DEBUG8) {
        Serial.print("Merc: true");
      }
      return true;
    }
  }
  if (DEBUG8) {
    Serial.print("Merc: false");
  }
  mFlag = false;
  return false;
}


void US_init() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

void startCounter() {
  duration = micros();
  detachInterrupt(US_INT_NUM);
  attachInterrupt(US_INT_NUM, stopCounter, FALLING);
}

void stopCounter() {
  duration = micros() - duration;
  distance = duration / 58;
  if (distance > 250)
    distance = 0;
  detachInterrupt(US_INT_NUM);
}

void motors_init() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
}

unsigned long US_TmpTime = 0;

void readUS() {
  if (millis() - US_TmpTime > 50) {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    attachInterrupt(US_INT_NUM, startCounter , RISING);
    US_TmpTime = millis();
  }
  if (DEBUG7) {
    Serial.print("Distance ");
    Serial.println(distance);
  }
}

void motors(int M1, int M2) {
  if (M1 > 100)
    M1 = 100;
  else if (M1 < -100)
    M1 = -100;

  if (M2 > 100)
    M2 = 100;
  else if (M2 < -100)
    M2 = -100;

  M1 = map(M1, -100, 100, -255, 255);
  M2 = map(M2, -100, 100, -255, 255);


  analogWrite(EN1, abs(M1));
  if (M1 > 0) {
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
  }
  else {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
  }

  analogWrite(EN2, abs(M2));
  if (M2 > 0) {
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
  }
  else {
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
  }
}

void stp() {
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, HIGH);
}




int getSens(int id) {
  if (id == SEN1) {
    return  map(analogRead(SEN1), SEN1_MAX, SEN1_MIN, 0, 100);
  }
  else if (id == SEN2) {
    return  map(analogRead(SEN2), SEN2_MAX, SEN2_MIN, 0, 100);
  }
  else if (id == SEN3) {
    return  map(analogRead(SEN3), SEN3_MAX, SEN3_MIN, 0, 100);
  }
  else if (id == SEN4) {
    return  map(analogRead(SEN4), SEN4_MAX, SEN4_MIN, 0, 100);
  }
}



/*
  #define LELEFT 1
  #define LERIGHT 2



  int lastSeen = 0; // 1 - left edge,  2 - left center,  3 - right center, 4 - right edge
  void lastSens() {
  if (S1 || S2 || S3 || S4)
  {

  }
  }


  int ELR = 0; //EDGE  1 -left, 2 -right  3 - both

  void lastEdge() {
  if (S1 || S4)
    if (S1 && S4)
      ELR = 3;
    else if (S1)
      ELR = 1;
    else
      ELR = 2;
  if (DEBUG5) {
    Serial.print(ELR);
    if (ELR == 1)
      Serial.println(" left");
    else if (ELR == 2)
      Serial.println(" right");
    else
      Serial.println(" both");
  }

  }
*/


void updateAnalog() {
  s1 = getSens(SEN1);
  s2 = getSens(SEN2);
  s3 = getSens(SEN3);
  s4 = getSens(SEN4);
}

void updateDigital() {
  if (s1 > BLK_VAL)
    S1 = true;
  else
    S1 = false;
  if (s2 > BLK_VAL)
    S2 = true;
  else
    S2 = false;
  if (s3 > BLK_VAL)
    S3 = true;
  else
    S3 = false;
  if (s4 > BLK_VAL)
    S4 = true;
  else
    S4 = false;
}

void updateSensors() {
  updateAnalog();
  updateDigital();
  MercSens();
  //lastEdge();
  //lastSens();

  if (DEBUG2) {
    Serial.print("S0:");
    Serial.print(S1);
    Serial.print( " S2:");
    Serial.print(S2);
    Serial.print(" S3:");
    Serial.print(S3);
    Serial.print(" S4:");
    Serial.println(S4);
  }
  if (DEBUG3) {
    Serial.print("s1:");
    Serial.print(s1);
    Serial.print( " s2:");
    Serial.print(s2);
    Serial.print(" s3:");
    Serial.print(s3);
    Serial.print(" s4:");
    Serial.println(s4);
  }
  if (DEBUG4) {
    Serial.print("s1:");
    Serial.print(analogRead(SEN1));
    Serial.print( " s2:");
    Serial.print(analogRead(SEN2));
    Serial.print(" s3:");
    Serial.print(analogRead(SEN3));
    Serial.print(" s4:");
    Serial.println(analogRead(SEN4));
  }
}

/*
  void DrvByLastEdge() {

  if (ELR == LELEFT) {
    motors(-baseSpeed * 0.3 , baseSpeed);
    while (!S1)
      updateSensors();

  }
  else if (ELR == LERIGHT) {
    motors(baseSpeed, -baseSpeed * 0.3);
    while (!S4)
      updateSensors();
  }
  else
    driveByErr();
  }
*/

void driveByErr(int state) {  // 1 - nornal , 2 - ramp
  static int error, left, right;
  error = s2 - s3;
  error *= 0.7;
  /*

    left = baseSpeed - error;
    right = baseSpeed + error;

    if((abs(error))>25)
    left*=2;
  */
  if (state == 1) {
    if (error >= 0) {
      left = baseSpeed - error * 1.2;
      right = baseSpeed * 0.8 + error;
    }
    else {
      left = baseSpeed * 0.8 - error;
      right = baseSpeed + error * 1.2;
    }
  }
  else if (state == 2) {
    if (error >= 0) {
      //left = baseSpeed - error * 1.2;
      right = baseSpeed * 3 + error;
    }
    else {
      left = baseSpeed * 3 - error;
      //right = baseSpeed + error * 1.2;
    }
  }


  motors(left, right);
  if (DEBUG1) {
    Serial.print("left:");
    Serial.print(left);
    Serial.print("right:");
    Serial.println(right);
  }
}


unsigned long tmp, ltime = 100;
boolean firstTime = true;

boolean ifLostLine() {
  if (!(s2 > 10 || s3 > 10)) {
    if (firstTime) {
      tmp = millis();
      firstTime = false;
      if (DEBUG6)
        Serial.println("firstTime");
    }
    if (millis() - ltime > tmp) {
      if (DEBUG6)
        Serial.println("LOST LINE");
      return true;
    }
  }
  else
    firstTime = true;

  return false;
}



void edgeDetected() {
  if (S1 && S4)
    motors(baseSpeed, baseSpeed);
  else if (S1) {
    motors(baseSpeed, -baseSpeed * 0.7);
    delay(40);
    while (!S1)
      updateSensors();
    motors(-baseSpeed, baseSpeed);
    while (!S3)
      updateSensors();
  }
  else {
    motors(-baseSpeed * 0.7, baseSpeed);
    delay(40);
    while (!S4)
      updateSensors();
    motors(baseSpeed, -baseSpeed);
    while (!S2)
      updateSensors();
  }

}

void drive() {
  /* if(ifLostLine())
    DrvByLastEdge();
    else
    driveByErr();*/
  if (MercSens())
    driveByErr(2);
  else
  {
    if (S1 || S4)
      edgeDetected();
    else
      driveByErr(1);
  }
}





void loop() {
  updateSensors(); 
  drive();
  /*for(int i=0;i<100;i=i+10)
  {
  setArmAngle(i);
  delay(500);
  }*/
  //motors(100,100);
}


