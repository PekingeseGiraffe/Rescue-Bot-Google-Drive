void setup()
{
  Serial.begin(115200);
}
void loop (){
  int rightSensorMid, leftSensorMid,rightSensorOut,leftSensorOut;
  rightSensorMid  = analogRead(A0);
  leftSensorMid  = analogRead(A3);
  rightSensorOut=analogRead(A2);
  leftSensorOut=analogRead(A1);
  
 rightSensorMid = map(rightSensorMid,520 , 70, 1000, 0);
 leftSensorMid = map(leftSensorMid, 880, 100,  1000, 0);
  rightSensorOut = map(rightSensorOut, 570, 60, 1000, 0);
  leftSensorOut = map(leftSensorOut, 465, 65,1000, 0);

  Serial.print(" RightSensorOut :");
  Serial.print(rightSensorOut);
  Serial.print(" RightSensor :");
  Serial.print(rightSensorMid);
  Serial.print(" LeftSensor :");
  Serial.print(leftSensorMid);
   Serial.print(" LeftSensorOut :");
  Serial.println(leftSensorOut);
  delay(10);
}
