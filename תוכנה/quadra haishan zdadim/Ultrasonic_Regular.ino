#define trig 10
#define echo 2

unsigned long Time = 0;
unsigned long duration;
int distance = 0;

void trigUs(){
  digitalWrite(trig,1);
  delayMicroseconds(10);
  digitalWrite(trig,0);
    duration = pulseIn(echo,HIGH);
  attachInterrupt(0,measurement ,RISING);
}


void measurement(){
  distance = duration/58;
  detachInterrupt(0);
}

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
  trigUs();
}


void loop() {
  Serial.print("Distance = ");
  Serial.println(distance);
  delay(100);
    trigUs();
}
