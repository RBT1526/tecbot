//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//cosas del color
#define commonAnode true
byte gammatable[256];
#define redpin 3
#define greenpin 5
#define bluepin 6
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void leerColor(){
  //rutina para leer color
  int IR = analogRead(A3);

  float red, green, blue;  
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  lcd.clear();
  lcd.setCursor(0,0);
  
  if(green>100 && IR>500){
    lcd.print("NEGRO");
  }else if(red>100){
    lcd.print("RED");    
  }else if(green>100){
    lcd.print("GREEN");
  }else if(blue>100){
    lcd.print("BLUE");
  }else 

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
}

void setup()
{
  Serial.begin(115200);
  tcs.begin();
  lcd.init();
  lcd.backlight();
  /*lcd.setCursor(0,0);
  lcd.print("Hello, world!");*/

  #if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
}

void loop()
{
  delay(100);
  leerColor();
}
