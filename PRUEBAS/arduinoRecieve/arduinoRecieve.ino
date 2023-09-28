#include "SerialUtil.h"

SerialUtil mySerial;

bool valueBool;
int valueInt=NAN;
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
  if(valueInt<=0){
    Serial.print("aun no ");
    mySerial.loop(millis()); 
    //recieveMyData('K');
    valueInt = mySerial.readInt();
  }else{
    Serial.print("fin ");
    Serial.println(valueInt);
  }
}

void sendMyData(){
  
}

void recieveMyData(int sendCode){  
  if (sendCode == 'K') {
    valueInt = mySerial.readInt();
    Serial.println(valueInt);
  }
}