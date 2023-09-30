#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;
float velDer=50,velIzq=50;

int vel_d = 100;
int vel_i = 100;
int vel_pid_d = 0;
int vel_pid_i = 0;


float target_angle;
float error;


float Kp = 1;


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
        /*Serial.println(heading);*/
        microsPrevious = microsPrevious + microsPerReading;
        }
    return heading;
}
void pid_check(float target){
    
    float angle_check = get_motion();

    error = target - angle_check;

    //Serial.println(error);
    vel_pid_i = vel_i - Kp*error;
    vel_pid_d = vel_d + Kp*error; 
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

}

void setup() {
    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);
    digitalWrite(standBy, HIGH);
    Serial.begin(115200);
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

  microsPerReading = 1000000 / 28;


    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
  microsPrevious = micros();
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
        filter.updateIMU(gx, gy, gz, ax, ay, az);

        
        target_angle = filter.getYaw();
}

void loop() {
    
    analogWrite(pwm_a, vel_pid_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_pid_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= 100000) {
    pid_check(target_angle);
 
    }

}

float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}