#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
unsigned long micros_inicio;

const int pwm_a = 9;
const int der_a = 8;
const int der_b = 7;
const int pwm_b = 3;
const int izq_a = 4;
const int izq_b = 5;
const int standBy = 6;
float velDer=80,velIzq=70;

int vel_d = 90;
int vel_i = 100;
int vel_pid_d = 0;
int vel_pid_i = 0;

bool flag = false;
float errores = 0;
float errores1 = 0;

float target_angle;
float error;
float error_ant;


float Kp = 2;




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
    
    if(error > 300){
        error = target - (360+angle_check);
    }
    if(error < -300){
        error = (angle_check-(360+target))*-1;
    }
    
    
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
    Serial.print(" angle: ");
    Serial.print(angle_check);
    Serial.print(" Target: ");
    Serial.println(target);

    
}/*
void pid_vuelta(float target){
    unsigned long micro_check, microspre,microcomple;
    micro_check = micros();
    microspre = micro_check;
    microcomple = micro_check;
    while(true){
    micro_check = micros();
    if(micro_check-microcomple >= 100000){
    float angle_check = get_motion();

    
    error = target - angle_check;
    
    if(error > 300){
        error = target - (360+angle_check);
    }
    if(error < -300){
        error = (angle_check-(360+target))*-1;
    }
    
    
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
    Serial.print("Error = ");
    Serial.print(error);
    Serial.print(" Error past= ");
    Serial.print(error_ant);
    Serial.print("Angle = ");
    Serial.println(angle_check);
    if ((error_ant < 1 || error_ant >= 0) && error_ant  == error){
        break;
    }
    if(micro_check - microspre >= 30000){
    error_ant = error;
    microspre = micro_check;
    }
    microcomple += 100000;
    }
    }
    //Serial.print("veld = ");
    //Serial.print(vel_pid_d);
    //Serial.print(" veli = ");
    //Serial.println(vel_pid_i);
}
void vuelta(float angle){
    target_angle += angle;
    int vel_ant_d = vel_d;
    int vel_ant_i = vel_i;
    vel_d = 0;
    vel_i = 0;
    Serial.println("YA ando aca");
    Serial.println(target_angle);
    //delay(2500);
    pid_vuelta(target_angle);
   // delay(2500);
    vel_d = vel_ant_d;
    vel_i = vel_ant_i;
    

}
*/

void turn(float targetAngle){
    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    float heading;

    unsigned long microsNow;

    microsNow = micros();

        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        heading = filter.getYaw();
        Serial.println(heading);
        
        targetAngle=heading+targetAngle;
        target_angle = targetAngle;
        if(targetAngle>=360){
            targetAngle=targetAngle-360;
        }else if(targetAngle<0){
            targetAngle=360+targetAngle;
        }

        Serial.println(heading);
        Serial.println(" ");
        Serial.println(targetAngle);

        bool si=false;

        if(heading+180<targetAngle){
            //giro der
            analogWrite(pwm_a,velDer);//right
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,HIGH);
            analogWrite(pwm_b,velIzq);
            digitalWrite(izq_a,HIGH);
            digitalWrite(izq_b,LOW);
            while(heading<targetAngle && si==false || heading>targetAngle && si==true){
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
                    Serial.println(heading);
                    microsPrevious = microsPrevious + microsPerReading;
                }
                if(heading>targetAngle){
                    si=true;
                }
            }
            //stop
            analogWrite(pwm_a,0);
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,0);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,LOW);
        }else if(heading-180>targetAngle){
            //giro izq
            analogWrite(pwm_a,velDer);//left
            digitalWrite(der_a,HIGH);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,velIzq);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,HIGH);          
            while(heading>targetAngle && si==false || heading<targetAngle && si==true){
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
                    Serial.println(heading);
                    microsPrevious = microsPrevious + microsPerReading;
                }
                if(heading<targetAngle){
                    si=true;
                }
            }
            //stop
            analogWrite(pwm_a,0);
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,0);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,LOW);
        }else if(heading<targetAngle){
            //giro izq
            analogWrite(pwm_a,velDer);//left
            digitalWrite(der_a,HIGH);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,velIzq);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,HIGH);          
            while(heading<targetAngle){
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
                    Serial.println(heading);
                    microsPrevious = microsPrevious + microsPerReading;
                }
            }
            //stop
            analogWrite(pwm_a,0);
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,0);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,LOW);
        }else if(heading>targetAngle){
            //giro der
            analogWrite(pwm_a,velDer);//right
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,HIGH);
            analogWrite(pwm_b,velIzq);
            digitalWrite(izq_a,HIGH);
            digitalWrite(izq_b,LOW);
            while(heading>targetAngle){
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
                    Serial.println(heading);
                    microsPrevious = microsPrevious + microsPerReading;
                }
            }
            //stop
            analogWrite(pwm_a,0);
            digitalWrite(der_a,LOW);
            digitalWrite(der_b,LOW);
            analogWrite(pwm_b,0);
            digitalWrite(izq_a,LOW);
            digitalWrite(izq_b,LOW);
            delay(2000);
        }
}

void setup() {
    Serial.begin(115200);
    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);
    digitalWrite(standBy, LOW);
    

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

microsPerReading = 1000000 / 28;
microsPrevious = micros();
micros_inicio = micros();
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
    //pid_check(358.0);
    }
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

float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}