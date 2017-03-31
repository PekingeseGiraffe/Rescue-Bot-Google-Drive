
void motors(int left, int right){
  if (left > 0)
  {
      digitalWrite(4,1);
      digitalWrite(5,0);  
      analogWrite(9, left);
  }
  
  else
   {
       digitalWrite(4,0);
      digitalWrite(5,1);  
      analogWrite(9, -left);
   }
   
    if (right > 0)
  {
      digitalWrite(6,1);
      digitalWrite(7,0);  
      analogWrite(8, right);
  }
  else
   {
       digitalWrite(6,0);
      digitalWrite(7,1);
      analogWrite(8, -right);
   }
}
