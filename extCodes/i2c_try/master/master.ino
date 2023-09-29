//Transmitter Code

 #include <SoftwareSerial.h>
 #include <QTRSensors.h>
 SoftwareSerial link(2, 3); // Rx, Tx

 QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


 char charVal[5];

  void setup() 
  {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(9);
    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
    link.begin(9600);
    Serial.begin(9600);
    delay(1000);
    
  }

  void loop()  
  {
   uint16_t position = qtr.readLineBlack(sensorValues);
   byte h = highByte(position);
  byte l = lowByte(position);
  link.write(h); //soft2.print(h, BIN);
  link.write(l);//soft2.print(l, BIN);
   delay(100);
  }
  
 /*
 #include<SoftwareSerial.h>
int info2 = 4000;//0x012C : 0000 0001 0010 1100

SoftwareSerial soft2(2, 3);


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  soft2.begin(9600);
}


void loop() 
{

  //SEND SOFT2 DATA
  byte h = highByte(info2);
  byte l = lowByte(info2);
  soft2.write(h); //soft2.print(h, BIN);
  soft2.write(l);//soft2.print(l, BIN);
  delay(200);

}
*/