#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

LiquidCrystal_I2C lcd(0x27,20,4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c;

void leerColor(){
  //rutina para leer color
  //int IR = analogRead(A3);//USAR QTR
  for(int i=0;i<5;i++){
    tcs.getRawData(&r, &g, &b, &c);
    delay(25);
  }
  Serial.print(r);
  Serial.print(" ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.println(c);
  lcd.clear();
  lcd.setCursor(0,0);  
  if(r<2000 && g<2000 && b<2000){//MODIFICAR
    lcd.print("NEGRO");
    Serial.println("n");
  }else if(r>4000 && g<4000 && b<4000){
    lcd.print("ROJO");
    Serial.println("r");
  }else if(r>3000 && g>9000 && b>4000){
    lcd.print("VERDE");
    Serial.println("v");
  }else if(r<3000 && g>3000 && b>5000){
    lcd.print("AZUL");
    Serial.println("a");
  }else if(r>10000 && g>13000 && b>4000){
    lcd.print("CHECK 1");
    Serial.println("1");
  }else if(r<6000 && g>13000 && b>13000){
    lcd.print("CHECK 2");
    Serial.println("2");
  }else if(r>5000 && g>7000 && b>6000){
    lcd.print("CHECK 3");
    Serial.println("3");
  }else if(r>4000 && b<5000 && b<6000){
    lcd.print("CHECK 4");
    Serial.println("4");
  }
  //return -2;
}

void setup(void) {
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}


void loop(void) {
  /*tcs.getRawData(&r, &g, &b, &c);
  Serial.print(r);
  Serial.print(" ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.println(c);
  delay(100);*/
  leerColor();
}