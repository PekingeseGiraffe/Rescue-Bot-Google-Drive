void setup()
{
  Serial.begin(9600);
}
void loop (){
  int rightSensor, leftSensor,midSensor;
  rightSensor  = analogRead(A0);
  leftSensor  = analogRead(A1);
  midSensor=analogRead(A2);
  Serial.print(" RightSensor :");
  Serial.print(rightSensor);
  Serial.print("  MidSensor:");
  Serial.print(midSensor);
  Serial.print(" LeftSensor :");
  Serial.println(leftSensor);
  delay(200);
}
