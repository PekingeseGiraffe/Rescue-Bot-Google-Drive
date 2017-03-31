#include <Servo.h> 
 
Servo kl,kr;
Servo zl,zr;  
 
void setup() 
{ 
  kr.attach(43); 
  kl.attach(45); 
  zl.attach(44); 
  zr.attach(42); 
} 
 
 
 void openZ(){
  kl.write(110);
  kr.write(70);
 }
 
  void closeZ(){
   kl.write(70);
   kr.write(100);
 }
 
 void moveUp(){
 zl.write(40);
 zr.write(140);
 }
 
  void moveDown(){
  zl.write(110);
 zr.write(70);
 }
 
void loop() 
{ 
moveUp();
delay(1000);
moveDown();
delay(1000);
} 
