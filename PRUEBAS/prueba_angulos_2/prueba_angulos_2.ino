#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {

  Serial.begin(115200);

  // start the IMU and filter

  CurieIMU.begin();

  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 0);

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

  int aix, aiy, aiz;

  int gix, giy, giz;

  float ax, ay, az;

  float gx, gy, gz;

  float roll, pitch, heading;

  unsigned long microsNow;

  // check if it's time to read data and update the filter

  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU

    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units

    ax = convertRawAcceleration(aix);

    ay = convertRawAcceleration(aiy);

    az = convertRawAcceleration(aiz);

    gx = convertRawGyro(gix);

    gy = convertRawGyro(giy);

    gz = convertRawGyro(giz);

    // update the filter, which computes orientation

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll

    roll = filter.getRoll();

    pitch = filter.getPitch();

    heading = filter.getYaw();

    Serial.print("Orientation: ");

    Serial.print(heading);

    Serial.print(" ");
    
    Serial.println(gx);


    

    // increment previous time, so we keep proper pace

    microsPrevious = microsPrevious + microsPerReading;

  }
}

float convertRawAcceleration(int aRaw) {

  // since we are using 2G range

  // -2g maps to a raw value of -32768

  // +2g maps to a raw value of 32767



  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) {

  // since we are using 250 degrees/seconds range

  // -250 maps to a raw value of -32768

  // +250 maps to a raw value of 32767



  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

/*void rotate (float targetAngle){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1){
    stopCar();
  } else {
    if (angle > targetAngle) { //turn left
      left();
    } else if (angle < targetAngle) {//turn right
      right();
    }

    //setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30){
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if (round(targetGyroX - abs(GyroX)) == 0){
      ;
    } else if (targetGyroX > abs(GyroX)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroX)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}*/