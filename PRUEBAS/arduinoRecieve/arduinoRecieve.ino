#include "SerialUtil.h"

SerialUtil mySerial;

bool valueBool;
int valueInt;
float valueFloat=NAN;
String valueStr;

void setup() {
  Serial.begin(9600);
  // Begin the Serial at 9600 Baud
  mySerial.setBaudrate(9600);
  mySerial.changeWaitTime(20);
  mySerial.startMode(MODE_RECIEVE);
  mySerial.comType(true,true);
  mySerial.attachSend(sendMyData);
  mySerial.attachRecieve(recieveMyData);
  mySerial.setTimeOut(2000);
  mySerial.setTimeChangeCom(5000);
  Serial.println("End setup");
}

void loop() {
  if(isnan(valueFloat)){
    Serial.print("aun no ");
    mySerial.loop(millis()); 
    recieveMyData('A');
  }else{
    Serial.print("fin ");
    Serial.println(valueFloat);
  }
}

void sendMyData(){
  //Time to send data    
  mySerial.sendFloat('a', 15.6);
  mySerial.sendFloat('b', 16.7);
  mySerial.sendFloat('c', 17.8);
  mySerial.sendInt('k', 10);
  mySerial.sendBool('p', false);
  mySerial.sendText('t', "Dit is een test van B naar A", 50);
}

void recieveMyData(int sendCode){  
  if (sendCode == 'A' || sendCode == 'B' || sendCode == 'C'
      || sendCode == 'D' || sendCode == 'E' || sendCode == 'F'
      || sendCode == 'G' || sendCode == 'H' || sendCode == 'I'
      || sendCode == 'J') {
    valueFloat = mySerial.readFloat();
    Serial.println(valueFloat);
  }
  else if (sendCode == 'K' || sendCode == 'L' || sendCode == 'M'
           || sendCode == 'N' || sendCode == 'O') {
    valueInt = mySerial.readInt();
    Serial.println(valueInt);
  }
  else if (sendCode == 'P' || sendCode == 'Q' || sendCode == 'R'
           || sendCode == 'S') {
    valueBool = mySerial.readBool();
    Serial.println(valueBool);
  }
  else if (sendCode == 'T') {
    valueStr = mySerial.readText(50);
    Serial.println(valueStr);
  }
}