#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
unsigned long micros_inicio;
unsigned long micros_prev;
const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 7;
const int izq_b = 8;
const int standBy = 6;
float velDer=80,velIzq=80;

int vel_d = 85;
int vel_i = 100;

float get_distance(){
    int s = analogRead(A0);
    float dist= pow(10,log10(s/1821.2)/-0.65);
    return dist;
}
float get_distance_b(){
    int s = analogRead(A1);
    float dist= pow(10,log10(s/1821.2)/-0.65);
    return dist;
}
float get_distance_c(){
    int s = analogRead(A2);
    float dist= pow(10,log10(s/1821.2)/-0.65);
    return dist;
}
/*
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
*/
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
}

void loop() {
    float distance_i = get_distance_b();
    float distance_d = get_distance_c();
    float distance_f = get_distance();
    analogWrite(pwm_a, vel_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    if(distance_d <= 4.0){
    analogWrite(pwm_a, vel_d+20);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    delay(300);
    }
    if(distance_i <= 4.0){
    analogWrite(pwm_a, vel_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_i+20);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    delay(300);
    }
    if(distance_f <= 5.0){
    analogWrite(pwm_a, 0);
    digitalWrite(der_a,LOW);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,0);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,LOW);
    }

}
