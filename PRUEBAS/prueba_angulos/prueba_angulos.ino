#include "CurieIMU.h"

void setup(){
    serial.begin(9600);
CurieIMU.begin();
CurieIMU.setGyroRange(250);
}

void loop(){
    int gxRaw,gyRaw,gzRaw;
    float gx,gy,gz;
    CurieIMU.readGyro(gxRaw,gyRaw,gzRaw);
    gx=convertRawGyro(gxRaw);
    gy=convertRawGyro(gyRaw);
    gz=convertRawGyro(gzRaw);

    Serial.print(gx," ",gy," ",gz);
}