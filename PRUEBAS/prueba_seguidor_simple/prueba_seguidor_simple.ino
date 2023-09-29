#include <Wire.h>
#include "Adafruit_TCS34725.h"


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

const int pwm_a = 9;
const int der_a = 8;
const int der_b = 7;
const int pwm_b = 3;
const int izq_a = 5;
const int izq_b = 4;
const int standBy = 6;

const int max_vel = 80;

int vel_d = 60;
int vel_i = 60;


void setup(void) {
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  pinMode(standBy, OUTPUT);
pinMode(pwm_a, OUTPUT);
pinMode(der_a, OUTPUT);
pinMode(der_b, OUTPUT);
pinMode(pwm_b, OUTPUT);
pinMode(izq_a, OUTPUT);
pinMode(izq_b, OUTPUT);
digitalWrite(standBy, LOW);
digitalWrite(izq_a,HIGH);
  digitalWrite(izq_b,LOW);
  digitalWrite(der_a,HIGH);
  digitalWrite(der_b,LOW);
  digitalWrite(standBy, HIGH);

}


void loop(void) {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  int realvalue = r;
  if(realvalue < 8000){
    analogWrite(pwm_a,vel_d);//front
    analogWrite(pwm_b,vel_i+30);
  }
  else if (realvalue > 8000){
    analogWrite(pwm_a,vel_d+30);//front
    analogWrite(pwm_b,vel_i);
  }
  else{
    analogWrite(pwm_a,vel_d);//front
    analogWrite(pwm_b,vel_i);   
  }
    /* code */
  
  
  Serial.print("Reading = ");
    Serial.print(realvalue);
    Serial.print("  vel_d = ");
    Serial.print(vel_d);
    Serial.print("  vel_i = ");
    Serial.println(vel_i);

    

}
