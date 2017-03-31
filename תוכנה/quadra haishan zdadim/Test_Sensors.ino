void setup()
{
  Serial.begin(9600);
}
void loop (){
  int rightSensorMid, leftSensorMid,rightSensorOut,leftSensorOut;
  rightSensorMid  = analogRead(A0);
  leftSensorMid  = analogRead(A3);
  rightSensorOut=analogRead(A2);
  leftSensorOut=analogRead(A1);

  
  rightSensorOut = map(rightSensorOut, 1000, 110, 1000, 0);
     rightSensorMid = map(rightSensorMid,1000 ,115, 1000, 0);
  leftSensorMid = map(leftSensorMid, 1000, 190,  1000, 0);
  leftSensorOut = map(leftSensorOut, 1000, 120,1000, 0);
  
  Serial.print(" RightSensorOut (A2):");
  Serial.print(rightSensorOut);
  Serial.print(" RightSensor (A0):");
  Serial.print(rightSensorMid);
  Serial.print(" LeftSensor (A3):");
  Serial.print(leftSensorMid);
   Serial.print(" LeftSensorOut (A1):");
  Serial.println(leftSensorOut);
  delay(200);
}
