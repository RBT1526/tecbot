#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°

void setup() {
  Serial.begin(115200);
  servos.begin();  
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms
}

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  servos.setPWM(n_servo, 0, duty);  
}

void loop() {  
    Serial.println("Servo 0");
    setServo(14,40);//14 elevador 40 abajo 180 arriba
    setServo(15,40);//15 garra 40 abierto 75 cerrado
    delay(2000);
    //setServo(14,40);//abrir
    //etServo(15,40);

    //setServo(14,40);//cerrar
    //setServo(15,75);
}