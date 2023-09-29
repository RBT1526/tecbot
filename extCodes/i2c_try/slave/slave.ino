//Receiver code
 #include <SoftwareSerial.h>
 SoftwareSerial link(11, 10); // Rx, Tx
  
  int c;

  void setup() 
  {
    link.begin(9600); //setup software serial
    Serial.begin(9600);    //setup serial monitord
  }

  void loop()  
  {  
 // if (link.available())
 //     Serial.write(link.read());
 // if (Serial.available())
 //     link.write(Serial.read());  
   link.listen();
  while (link.available() < 1) {}

  if (link.available() > 1) 
  {
    Serial.println("Data from port two:");
    byte h = link.read();       //read those two bytes back to back!
    byte l = link.read();
    c = (h << 8) + l;
    Serial.println(c);
  }
   /*
   while(link.available())
   {
    //read incoming char by char:
     ch = link.read();
     cString[chPos] = ch;
     chPos++;     
     
   digitalWrite(greenLED, HIGH); //flash led to show data is arriving
   delay(10);
   digitalWrite(greenLED, LOW);
   }
   cString[chPos] = 0; //terminate cString
   chPos = 0;
   Serial.print(cString);
   */
  }