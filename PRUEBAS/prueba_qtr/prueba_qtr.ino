#include <QTRSensors.h>

QTRSensors qtr;

const int pwm_a = 9;
const int der_a = 7;
const int der_b = 8;
const int pwm_b = 3;
const int izq_a = 5;
const int izq_b = 4;
const int standBy = 6;
float velDer=150,velIzq=150;

void setup()
{
    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);

digitalWrite(standBy, HIGH);

  qtr.setSensorPins((const uint8_t[]){A0, A1, A2}, 3);
  for (uint8_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }
}
void loop()
{
  int16_t position = qtr.readLineBlack(sensors);

  int16_t error = position - 1000;

    analogWrite(pwm_a,velDer);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,velIzq);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,HIGH);

  if (error < -500)
  {
    // the line is on the left
    velIzq-=0.01;
  }else if (error > 500)
  {
    // the line is on the right
    velDer-=0.01;
  }else{
    if(velIzq<150){
        velIzq+=1;
    }else if(velDer<150){
        velDer+=1;
    }
  }

  // set motor speeds using the two motor speed variables above
}