#include "CurieIMU.h"
int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

int gxBrightness = 0;
int gxLed = 9;

int gyBrightness = 0;
int gyLed = 6;


int gzBrightness = 0;
int gzLed = 5;

void setup(){
  
  pinMode(gxLed, OUTPUT); 
  pinMode(gyLed, OUTPUT); 
  pinMode(gzLed, OUTPUT); 
  Serial.begin(9600); // initialize Serial communication
  CurieIMU.begin();
  delay(5000); // Allow the user to set everything down
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 0);
}

String jsonEncodeValue(String key, float keyVal){
  return "\"" + key + "\":" + String(keyVal) + "";
}

String assembleJson(String keysAndVals){
  return "{" + keysAndVals + "}";
}

void loop(){
  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

  // display JSON formatted accel/gyro x/y/z values
  String keyVals = jsonEncodeValue("ax", ax) + ",";
  keyVals += jsonEncodeValue("ay", ay) + ",";
  keyVals += jsonEncodeValue("az", az) + ",";
  keyVals += jsonEncodeValue("gx", gx) + ",";
  keyVals += jsonEncodeValue("gy", gy) + ",";
  keyVals += jsonEncodeValue("gz", gz);
  
  if(Serial){
    Serial.println(keyVals);
  }
  delay(100);
/*
  if(gx>0)
    gxBrightness = gx/66.66;
  else
    gxBrightness=0;
  analogWrite(gxLed,gxBrightness);

  if(gy>0)
    gyBrightness = gy/66.66;
  else
    gyBrightness=0;
  analogWrite(gyLed,gyBrightness);
  

  
  if(gz>0)
    gzBrightness = gz/66.66;
  else
    gzBrightness=0;
  analogWrite(gzLed,gzBrightness);
*/
}
