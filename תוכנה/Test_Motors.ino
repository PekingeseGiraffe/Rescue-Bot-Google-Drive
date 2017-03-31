
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



void setup(){
  for(int i = 4; i < 8; i++)
     pinMode(i, OUTPUT); 
}


void loop(){
    motors(100,100);
    
    delay(2000);
    
    motors(-100,-100);
    
      delay(2000);
      
      motors(0,100);
      
      delay(2000);
      
      motors(100,0);
      
      delay(2000);
      
      motors(100,-100);
      
      delay(2000);
      
      motors(-100,100);
      
        delay(2000);
        
        motors(60,60);
        
        delay(2000);
        
        motors(-60,-60);
        
        delay(2000);
        
        motors(0,0);
        
        delay(1000);
}
