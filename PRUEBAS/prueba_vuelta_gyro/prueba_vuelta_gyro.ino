#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

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
        if(targetAngle>=360){
            targetAngle=targetAngle-360;
        }else if(targetAngle<0){
            targetAngle=360+targetAngle;
        }

        Serial.println(heading);
        Serial.println(" ");
        Serial.println(targetAngle);

        if(heading<targetAngle){
            //giro izq
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
        }else if(heading>targetAngle){
            //giro der
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
        }


    return;
}

void setup() {

  Serial.begin(9600);

  // start the IMU and filter

  CurieIMU.begin();
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  CurieIMU.setGyroRate(25);

  CurieIMU.setAccelerometerRate(25);

  filter.begin(25);

  // Set the accelerometer range to 2G

  CurieIMU.setAccelerometerRange(2);

  // Set the gyroscope range to 250 degrees/second

  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate

  microsPerReading = 1000000 / 25;

  microsPrevious = micros();
}

void loop() {
    delay(8000);
    turn(90);
    Serial.println("regreso");
}

float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}