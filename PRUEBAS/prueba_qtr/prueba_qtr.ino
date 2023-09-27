#include <QTRSensors.h>

QTRSensors qtr;
uint16_t sensors[8];

#define EMITTER_PIN 9

const int pwm_a = 9;
const int der_a = 7;
const int der_b = 8;
const int pwm_b = 3;
const int izq_a = 5;
const int izq_b = 4;
const int standBy = 6;
float velDer=150,velIzq=150;
float speed=150;


float Kp = 0;//edit
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;//edit
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;
int P, D, I, previousError, PIDvalue, error;

void motor_drive(){
    analogWrite(pwm_a,velDer);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,velIzq);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,HIGH);
}

void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    velIzq = speed - PIDvalue;
    velDer = speed + PIDvalue;

    if (velIzq > 255) {
      velIzq = 255;
    }
    if (velIzq < -255) {
      velIzq = -255;
    }
    if (velDer > 255) {
      velDer = 255;
    }
    if (velDer < -255) {
      velDer = -255;
    }
    motor_drive();
}

void setup()
{
Serial.begin(9600);

    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);

digitalWrite(standBy, HIGH);
     qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);
  qtr.setEmitterPin(9);
  for (uint8_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }
}
void loop()
{
    
  int16_t position = qtr.readLineBlack(sensors);

  int16_t error = 2000 - position;
    Serial.println(error);
    delay(200);
    PID_Linefollow(error);
}