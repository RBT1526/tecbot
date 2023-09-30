 /*
  Rotary Encoder Demo
  rot-encode-demo.ino
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
 // Rotary Encoder Inputs
 #define inputCLK 5
 #define inputDT 6
 
 // LED Outputs

 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK; 

 String encdir ="";

 void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);
   
   // Set LED pins as outputs
   
   // Setup Serial Monitor
   Serial.begin (9600);
   
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK = digitalRead(inputCLK);

 } 

 void loop() { 
  
  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
       
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter --;
       encdir ="CCW";
      
       
     } else {
       // Encoder is rotating clockwise
       counter ++;
       encdir ="CW";
       
       
     }

     Serial.println(counter);
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
 }