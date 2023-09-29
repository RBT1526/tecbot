#include <Servo.h>
Servo servo;
int angle = 180;//80 cerrado 0 abierto 60 abajo 180 arriba max
void setup() {
  Serial.begin(9600);
  servo.attach(8);
  servo.write(angle);
}
void loop() 
{ 
 // scan from 0 to 180 degrees
  /*for(angle = 0; angle < 180; angle++)  
  {                                  
    servo.write(angle);
    Serial.println(angle);               
    delay(15);                   
  } 
  // now scan back from 180 to 0 degrees
  for(angle = 180; angle > 0; angle--)    
  {                                
    servo.write(angle);    
    Serial.println(angle);       
    delay(15);       
  } */
}