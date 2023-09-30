#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

Madgwick filter;
unsigned long microsPerReading, microsPrevious, prev_millis;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

LiquidCrystal_I2C lcd(0x27,20,4);

const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;
float velDer=80,velIzq=70;

uint16_t r, g, b, c;

int vel_d = 100;
int vel_i = 90;
int vel_pid_d = 0;
int vel_pid_i = 0;


float target_angle;
float error;
float error_ant;


float Kp = 1;
float Kd = 0.01;
float Ki = 1;




















float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

float get_motion(){
    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    float heading;
    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        heading = filter.getYaw();
        microsPrevious = microsPrevious + microsPerReading;
        }
    return heading;
}

void pid_check(float target){
    
    float angle_check = get_motion();
    
    
    error = target - angle_check;
    
    if(error > 300){
        error = target - (360+angle_check);
    }
    if(error < -300){
        error = (angle_check-(360+target))*-1;
    }
    
    
    vel_pid_i = vel_i - (Kp*error+Kd*(error-error_ant)+Ki*(error+error_ant));//i+error
    vel_pid_d = vel_d + (Kp*error+Kd*(error-error_ant)+Ki*(error+error_ant)); 
    error_ant = error;

    if (vel_pid_i > 255) {
        vel_pid_i = 255;
    }
    if (vel_pid_i < 0) {
        vel_pid_i = 0;
    }
    if (vel_pid_d > 255) {
        vel_pid_i = 255;
    }
    if (vel_pid_d < 0) {
        vel_pid_d = 0;
    }
    digitalWrite(standBy, HIGH);
    /*
    Serial.print("veld = ");
    Serial.print(vel_pid_d);
    Serial.print(" veli = ");
    Serial.print(vel_pid_i);
    Serial.print("angle = ");
    Serial.print(angle_check);
    Serial.print(" target = ");
    Serial.print(target);
    Serial.print(" error = ");
    Serial.println(error);
    */

}

float get_distance(int port){

    int s = analogRead(port);
    float dist= pow(10,log10(s/1821.2)/-0.65);
    return dist;
}
























void setup() {

    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);
    digitalWrite(standBy, LOW);
    Serial.begin(115200);

    tcs.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();


    CurieIMU.begin();
    CurieIMU.autoCalibrateGyroOffset();
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    CurieIMU.setGyroRate(25);
    CurieIMU.setAccelerometerRate(25);
    filter.begin(25);
    CurieIMU.setAccelerometerRange(2);
    CurieIMU.setGyroRange(250);

    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    target_angle = filter.getYaw();

    microsPrevious = micros();
    microsPerReading = 1000000 / 28;


    lcd.setCursor(0,0);
    lcd.print("CAUTINES CALIENTES");

}

void checar_color(){
    tcs.getRawData(&r, &g, &b, &c);
    /*
    Serial.print("R: "); Serial.print(r); Serial.print(" ");
    Serial.print("G: "); Serial.print(g); Serial.print(" ");
    Serial.print("B: "); Serial.print(b); Serial.print(" ");
    Serial.print("C: "); Serial.print(c); Serial.print(" ");
    Serial.println(" ");
    */
    if(r>g && r>b){
        lcd.setCursor(0,1);
        lcd.print("ROJO");
    }
    if(g>r && g>b){
        lcd.setCursor(0,1);
        lcd.print("VERDE");
    }
    if(b>r && b>g){
        lcd.setCursor(0,1);
        lcd.print("AZUL");
    }
    if(r>g && r>b && c>30000){
        lcd.setCursor(0,1);
        lcd.print("BLANCO");
    }
    if(g>r && g>b && c>30000){
        lcd.setCursor(0,1);
        lcd.print("AMARILLO");
    }
    if(b>r && b>g && c>30000){
        lcd.setCursor(0,1);
        lcd.print("AZUL");
    }   
}

void avanzar(){
analogWrite(pwm_a, vel_pid_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_pid_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    if (micros() - microsPrevious >= 50000) {
        pid_check(target_angle);
        microsPrevious = micros();
    }
}
void stop(){
    analogWrite(pwm_a, 0);
    digitalWrite(der_a,LOW);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,0);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,LOW);
}



void loop() { 
    
    if(millis() -  prev_millis >= 680){
    stop();
    checar_color();
    delay(2000);
    prev_millis = millis();
    }
    avanzar();
    

    /*
    if(microsNow - micros_inicio >= 2000000){
        analogWrite(pwm_a,0);
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,0);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,LOW);
            delay(2000);
        turn(90);
        micros_inicio = micros();
    }
    */
}

//R: 4719 G: 13165 B: 5745 C: 25020 verde
// R: 14079 G: 26663 B: 20304 C: 64119 blanco
// R: 8873 G: 12914 B: 4771 C: 28496 amarillo
// R: 5729 G: 2581 B: 2214 C: 10446 rojo