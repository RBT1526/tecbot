/*#include <QTRSensors.h>

QTRSensors qtr;
uint16_t sensors[8];

#define EMITTER_PIN 9*/

void setup()
{
Serial.begin(9600);
/*qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);
  qtr.setEmitterPin(9);
  for (uint8_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }*/
}
void loop()
{
    /*mySerial.loop(millis()); 
    int16_t position = qtr.readLineBlack(sensors);
    int16_t error = 1000 - position;*/
    
}