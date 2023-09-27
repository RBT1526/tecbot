#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//cosas del sensor de color

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void leerColor(){
  //rutina para leer color
  float red, green, blue;  
  tcs.setInterrupt(false);  // turn on LED
  tcs.getRGB(&red, &green, &blue);
  
  lcd.clear();
  lcd.setCursor(0,0);  
  if(red>100){
    lcd.print("RED");    
  }else if(green>100){
    lcd.print("GREEN");
  }else if(blue>100){
    lcd.print("BLUE");
  } 
  

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.println(int(blue));
}

void setup()
{
  Serial.begin(115200);
  tcs.begin();
  lcd.init();
  lcd.backlight();


}

void loop()
{ //hola
  leerColor();
  delay(100);
}
