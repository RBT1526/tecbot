#include <Wire.h>//I2C communication
#include <LiquidCrystal_I2C.h>//LCD
#include "Adafruit_TCS34725.h"//sensor rgb TCS34725
#include <CurieIMU.h>//giroscopio
#include <MadgwickAHRS.h>//PID
#include <Adafruit_PWMServoDriver.h>//servo

//declarar servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°

//IMU
Madgwick filter;
unsigned long microsPerReading, microsPrevious,lastMicros;
float accelScale, gyroScale;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//cosas del sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c;

//PID
unsigned long micros_inicio;
unsigned long micros_prev;
const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;
float velDer=80,velIzq=70;
int vel_d = 100;
int vel_i = 90;
int vel_pid_d = 80;
int vel_pid_i = 80;
bool flag = false;
float errores = 0;
float errores1 = 0;
float target_angle;
float error;
float error_ant;
float Kp = 1;
float Kd = 0.01;
float Ki = 1;

int path[20][2];
int soli=0;
int colores[3]={0,0,0}; // 0 azul 1 rojo 2 verde
int targetColor=2;

int leerColor(){
  //rutina para leer color
  //int IR = analogRead(A3);//USAR QTR
  for(int i=0;i<5;i++){
    tcs.getRawData(&r, &g, &b, &c);
    delay(25);
  }
  lcd.clear();
  lcd.setCursor(0,0);  
  if(r<2000 && g<2000 && b<2000){//MODIFICAR
    lcd.print("NEGRO");
    return 5;
  }else if(r>4000 && g<4000 && b<4000){
    colores[1]+=1;
    lcd.print("ROJO");
    return 3;    
  }else if(r>3000 && g>9000 && b>4000){
    colores[2]+=1;
    lcd.print("VERDE");
    return 4;
  }else if(r<3000 && g>3000 && b>5000){
    colores[0]+=1;
    lcd.print("AZUL");
    return 2;
  }
  return -2;

  /*Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));*/
}
//arreglo (y, x) 1 amarillo 2  azul 3 rojo 4 verde 5 negro
int mazeColores[7][5] = {
    {-1, -1, -1, -1, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 1, 0, -1},
    {-1, -1, -1, -1, -1}
};
int mazeVisitados[7][5] = {
    {-1, -1, -1, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, -1, -1, -1, -1}
};
//arreglo tridimensional [4] para guardar si hay pared tmb guardar si el camino esta libre
int mazeParedes[4][7][5]={{
    {-1, -1, -1, 1, -1},
    {-1, 1, 1, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, -1, -1, -1, -1}
},{
    {-1, -1, -1, 1, -1},
    {-1, 0, 0, 1, -1},
    {-1, 0, 0, 1, -1},
    {-1, 0, 0, 1, -1},
    {-1, 0, 0, 1, -1},
    {-1, 0, 0, 1, -1},
    {-1, -1, -1, -1, -1}
},{
    {-1, -1, -1, 2, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 0, 0, 0, -1},
    {-1, 1, 1, 1, -1},
    {-1, -1, -1, -1, -1}
},{
    {-1, -1, -1, 1, -1},
    {-1, 1, 0, 0, -1},
    {-1, 1, 0, 0, -1},
    {-1, 1, 0, 0, -1},
    {-1, 1, 0, 0, -1},
    {-1, 1, 0, 0, -1},
    {-1, -1, -1, -1, -1}
}};
//direcciones (up, right, down, left)
int dx[] = {0, 1, 0, -1};
int dy[] = {-1, 0, 1, 0};
//orientacion del robot (0, 90, 180, 270)
int orientacion=0;
//posicion del robot
int xRobot=2;
int yRobot=5;
//motores
/*const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 7;
const int izq_b = 8;
const int standBy = 6;
float velDer=45,velIzq=45;
//PID LINE FOLLOWER
float Kp = 1;//edit
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;//edit
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;
int P, D, I, previousError, PIDvalue, error;*/
//cosas para calcular angulos
float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
void motor_drive(int v1,int v2){
    analogWrite(pwm_a,v1);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,v2);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,HIGH);
}
//girar a angulo
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

        Serial.print(heading);
        Serial.print(" ");
        Serial.println(targetAngle);

         bool si=false;

        if(heading+180<targetAngle && heading<180){
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
        }else if(heading-180>targetAngle && heading>180){
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
        }
    return;
}
void orientar(int o){
    if(orientacion==0 && o==270){
        //girar izquierda
        turn(90);
        orientacion=270;
    }else if(orientacion==270 && o==0){
        //girar derecha
        turn(-90);
        orientacion=0;
    }else if(orientacion<o){
        //girar derecha
        turn(-90);
        orientacion+=90;
        orientar(o);
    }else if(orientacion>o){
        //girar izquierda
        turn(90);
        orientacion-=90;
        orientar(o);
    }
    return;
}
void center(int a,int b){
    int oInicial=orientacion;
    orientar(a);
    int distFront;
    //roll forward
    do{
    int front = analogRead(A0);
    distFront= pow(10,log10(front/1821.2)/-0.65);
    }while(distFront>43);
    //stop
    if(b!=-1){
        orientar(b);
        //roll forward
        do{
        int front = analogRead(A0);
        distFront= pow(10,log10(front/1821.2)/-0.65);
        }while(distFront>43);
        //stop
    }
    orientar(oInicial);
    return;
}
//moverme hacia coordenada
void moveTo(int x1,int y1,int x2,int y2,float d){
    if(x1==x2){
        //mover en y
        if(y1>y2){
            orientar(0);
        }else{
            orientar(180);
        }
    }else{
        //mover en x
        if(x1>x2){
            orientar(270);
        }else{
            orientar(90);
        }
    }
    avanzar(d);
    xRobot=x2;
    yRobot=y2;
    return;
}
// recorrer path
void goTo(int x1,int y1){ //parametros = posicion inicial
    for(int i=0;i<soli;i++){
        moveTo(x1,y1,path[i][0],path[i][1],30);
        x1=path[i][0];
        y1=path[i][1];
    }
    xRobot=path[soli-1][0];
    yRobot=path[soli-1][1];
    return;
}
//recorrer path inverso
void goBack(int x1,int y1){ //parametros = posicion inicial
    for(int i=soli-2;i>=0;i--){
        moveTo(x1,y1,path[i][0],path[i][1],30);
        x1=path[i][0];
        y1=path[i][1];

    }
    xRobot=path[0][0];
    yRobot=path[0][1];
    return;
}

//función recursiva para checar cada cuadro posible
void scanMaze(int x,int y){
    for(int i=0;i<4;i++){ //checa muros
      if(mazeParedes[i][y][x]==0){
          //CHECAR CUAL SENSOR USAR
          int prevO=orientacion;
          int s;
          orientar(0);
          if(i==0){
            s = analogRead(A3);//front
          }else if(i==1){
            s = analogRead(A1);//der
          }else if(i==2){
            s = analogRead(A2);//back
          }else{
            s = analogRead(A0);//izq
          }
          orientar(prevO);
          int dist= pow(10,log10(s/1821.2)/-0.65);
          if(dist<6){
              //1 si hay pared
              /*lcd.clear();
              lcd.setCursor(0,0);
              lcd.print(i);
              lcd.setCursor(2,0);
              lcd.print("hay pared");*/
              mazeParedes[i][y][x]=1;
          }else{
              //2 si no hay
              /*lcd.clear();
              lcd.setCursor(0,0);
              lcd.print(i);
              lcd.setCursor(2,0);
              lcd.print("no hay pared");*/
              mazeParedes[i][y][x]=2;
          }
          /*lcd.setCursor(0,1);
          lcd.print(dist);
          delay(2000);*/
        }
    }
  for(int i=0;i<4;i++){ //moverme a cada direccion
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeColores[y2][x2]!=-1 && mazeColores[y2][x2]!=5 && mazeColores[y2][x2]==0 && mazeParedes[i][y][x]==2){
        if(soli>0){
            goTo(xRobot,yRobot);
            soli=0;
        }
        moveTo(xRobot,yRobot,x2,y2,15); //moverme 15 y checar color
        mazeColores[y][x]=leerColor();
        if(mazeColores[y][x]==5){ //moverme el resto o regresar
            moveTo(xRobot,yRobot,x2,y2,-15);
        }else{
            moveTo(xRobot,yRobot,x2,y2,15);
            scanMaze(x2,y2);
            path[soli][0]=x;
            path[soli][1]=y;
            soli++;
        }
    }
  }
  return;
}
bool fin=false;
void solveMaze(int x,int y){ //encontrar el camino mas corto
    if(fin==true){
        return;
    }
    if(x==3 && y==0){ //checar si llegue al final
        fin=true;
        return;
    }
    mazeVisitados[y][x]=1;
  for(int i=0;i<4 && fin==false;i++){ //interntar moverme a todos los lados
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeVisitados[y2][x2]!=-1 && mazeColores[y2][x2]!=5 && mazeVisitados[y2][x2]==0 && mazeParedes[i][y][x]==2){
        path[soli][0]=x2;
        path[soli][1]=y2;
        soli++;
      solveMaze(x2,y2);
      if(fin==false){
        soli--;
      }
    }
  }
  return;
}
void findColor(int x,int y){ //encontrar el camino hacia el color
    if(fin==true){
        return;
    }
    if(mazeColores[y][x]==targetColor){ //checar si estoy en el color
        fin=true;
        return;
    }
    mazeVisitados[y][x]=1;
  for(int i=0;i<4 && fin==false;i++){ // intenter moverme a todos lados
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeVisitados[y2][x2]!=-1 && mazeColores[y2][x2]!=5 && mazeVisitados[y2][x2]==0 && mazeParedes[i][y][x]==2){
        path[soli][0]=x2;
        path[soli][1]=y2;
        soli++;
      findColor(x2,y2);
      if(fin==false){
        soli--;
      }
    }
  }
  return;
}
void downRamp(){
    center(0,90);
    orientar(270);
    int distFront;
    //roll forward
    do{
        int der = analogRead(A0);
        int izq = analogRead(A1);
        int front = analogRead(A3);
        int distDer= pow(10,log10(der/1821.2)/-0.65); //cambiar pines
        int distIzq= pow(10,log10(izq/1821.2)/-0.65);
        distFront= pow(10,log10(front/1821.2)/-0.65);
        if(distDer<=4){
            //subir velocidad motor der
        }else if(distIzq<=4){
            //subir velocidad motor izq
        }
    }while(distFront>43);
    //stop
    center(0,270);
    orientar(90);
    //abrir garra
    orientar(180);
    return;
}
void leaveB(int x,int y){ //encontrar el camino mas corto
    if(fin==true){
        return;
    }
    if(x==3 && y==5){ //checar si llegue al final
        fin=true;
        return;
    }
    mazeVisitados[y][x]=1;
  for(int i=0;i<4 && fin==false;i++){ //interntar moverme a todos los lados
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeVisitados[y2][x2]!=-1 && mazeColores[y2][x2]!=4 && mazeVisitados[y2][x2]==0){
        path[soli][0]=x2;
        path[soli][1]=y2;
        soli++;
      leaveB(x2,y2);
      if(fin==false){
        soli--;
      }
    }
  }
  return;
}
int cubosColocados=0;
bool tengoCubo=false;
int coloresB(){
    //rutina para leer color zona B
  //int IR = analogRead(A3);//USAR QTR
  for(int i=0;i<5;i++){
    tcs.getRawData(&r, &g, &b, &c);
    delay(25);
  }
  //encontrar color
  if(r>4000 && g<4000 && b<4000){
    return 1;    
  }else if(r>3000 && g>9000 && b>4000){
    return 2;
  }
  return 3;
}
void solveB(int x,int y){
    //1 rojo
    //2 verde
    //3 vacio
    for(int i=0;i<4 && cubosColocados<3;i++){ //moverme a cada direccion
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeColores[y2][x2]!=-1 && mazeVisitados[y2][x2]==0 && cubosColocados<3){
        mazeVisitados[y2][x2]=1;
        moveTo(xRobot,yRobot,x2,y2,17.5); //moverme 15 y checar color
        if(mazeColores[y2][x2]==0){
          mazeColores[y2][x2]=coloresB();
        }
        if(mazeColores[y2][x2]==1 && tengoCubo==false){
            //lo agarro
            mazeColores[y2][x2]=3;
            tengoCubo=true;
            setServo(14,40);//cerrar
            delay(500);
            setServo(15,75);
            moveTo(xRobot,yRobot,x2,y2,12.5);
            solveB(x2,y2);
            moveTo(xRobot,yRobot,x,y,-30);
            //fin=false;
            //findGreen();
            //goTo(x2,y2);
        }else if(mazeColores[y2][x2]==2 && tengoCubo==true){
            //lo dejo
            cubosColocados+=1;
            mazeColores[y2][x2]=4;//ya no voy a este cuadro
            setServo(14,40);//abrir
            delay(500);
            setServo(15,40);
            moveTo(xRobot,yRobot,x2,y2,-17.5);
        }else if(mazeColores[y2][x2]==3){
            moveTo(xRobot,yRobot,x2,y2,12.5);            
            solveB(x2,y2);
            moveTo(xRobot,yRobot,x,y,-30);
        }
    }
  }
    return;
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
/*void moveForward(float target,float d){//se cancela
  unsigned long millisPrev=millis();
  unsigned long millisNow=millis();
  do{
    millisNow=millis();
    analogWrite(pwm_a, vel_pid_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_pid_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
  digitalWrite(standBy, HIGH);

    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= 100000) {
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
    }
  }while(millisNow-millisPrev<d*23);//689 = 30cm
  analogWrite(pwm_a,0);//apagar motores
  digitalWrite(der_a,LOW);
  digitalWrite(der_b,LOW);
  analogWrite(pwm_b,0);
  digitalWrite(izq_a,LOW);
  digitalWrite(izq_b,LOW);
}*/
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
void avanzar(float d){
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print("jala");
    analogWrite(pwm_a, vel_pid_d);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,vel_pid_i);
    digitalWrite(izq_a,HIGH);
    digitalWrite(izq_b,LOW);
    unsigned long lastMicros=micros();
    unsigned long microsYa=micros();
    target_angle=get_motion();
    while(microsYa-lastMicros<1000000*d/16){
      if (micros() - microsPrevious >= 50000) {
          pid_check(target_angle);
          microsPrevious = micros();
      }      
      microsYa=micros();
    }
    stop();
    lcd.clear();
  lcd.setCursor(1,0);  
  lcd.print("termino");
}
void stop(){
    analogWrite(pwm_a, 0);
    digitalWrite(der_a,LOW);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,0);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,LOW);
}
bool lastDir=true,wasWhite=true;//der
int angleSum=0,checks=0;
void lineFollower(){
  tcs.getRawData(&r, &g, &b, &c);
  //int headingInicial=get_motion();
  int plus=1;  
  while(r>6000 && g>14000 && b>7000){
    tcs.getRawData(&r, &g, &b, &c);
    turn(plus);
    angleSum+=plus;
    if(angleSum>90 || angleSum<-90){
      plus*=-1;
      delay(25);
    }
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print(angleSum);
    if(wasWhite==true && r<12000 && g<23000 && b>17000){// && rgb == plat
      checks++;
      wasWhite=false;
    }
    if(r>12000 && g>23000 && b>17000){//rgb == white
      wasWhite=true;
    }
  }
  lcd.clear();
  lcd.setCursor(1,0);  
  lcd.print("avanza bro");
  analogWrite(pwm_a, 100);
  digitalWrite(der_a,HIGH);
  digitalWrite(der_b,LOW);
  analogWrite(pwm_b,100);
  digitalWrite(izq_a,HIGH);
  digitalWrite(izq_b,LOW);
  delay(250);
  stop();
  if(checks<3){
    lineFollower();
  }
}
int zonaColor(){
    //rutina para leer color de checkpoint
  //int IR = analogRead(A3);//USAR QTR
  for(int i=0;i<5;i++){
    tcs.getRawData(&r, &g, &b, &c);
    delay(25);
  }
  lcd.clear();
  lcd.setCursor(0,0);
    //checar el color (amarillo 1, azul claro 2, rosa 3, violeta 4)
  if(r>10000 && g>13000 && b>4000){
    lcd.print("CHECK 1");
    Serial.println("1");
    return 1;
  }else if(r<6000 && g>13000 && b>13000){
    lcd.print("CHECK 2");
    Serial.println("2");
    return 2;
  }else if(r>5000 && g>7000 && b>6000){
    lcd.print("CHECK 3");
    Serial.println("3");
    return 3;
  }else if(r>4000 && b<5000 && b<6000){
    lcd.print("CHECK 4");
    Serial.println("4");
    return 4;
  }
  return 0;
}
void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  servos.setPWM(n_servo, 0, duty);  
}
void setup(){
    Serial.begin(9600);//115200
    //lcd pantalla
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("setup");
    //motores
    pinMode(standBy, OUTPUT);
    pinMode(pwm_a, OUTPUT);
    pinMode(der_a, OUTPUT);
    pinMode(der_b, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    pinMode(izq_a, OUTPUT);
    pinMode(izq_b, OUTPUT);
    digitalWrite(standBy, HIGH);
  //cosas servos
  servos.begin();
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms
  //cosas del IMU
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
  //cosas del rgb  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print("setupFin");
  lastMicros=micros();
}
  
void loop(){
  /*int z=zonaColor();
  z=4;
  if(z==1){
    //rutina zona a
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("ZONA A");
    setServo(14,180);//guardar
    setServo(15,60);
    scanMaze(2,5);
    soli=0;
    solveMaze(xRobot,yRobot);
    goTo(xRobot,yRobot);
    soli=0;
    if(colores[1]==5){
      targetColor=3;
    }else if(colores[2]==5){
      targetColor=4;
    }
    fin=false;
    for(int i=1;i<6;i++){
      for(int j=1;j<4;j++){
          mazeVisitados[i][j]=0;
      }
    }
    findColor(3,0);
    goTo(3,0);
    goBack(xRobot,yRobot);
    soli=0;
    }else if(z==2){
    //rutina rampa
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("RAMPA");
    setServo(14,40);//cerrar
    delay(500);
    setServo(15,75);
    downRamp();
  }else if(z==3){
    //rutina zona b
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("ZONA B");
    for(int i=1;i<6;i++){
      for(int j=1;j<4;j++){
          mazeColores[i][j]=0;
      }
    }
    mazeColores[6][3]=-1;
    xRobot=2;
    yRobot=0;
    do{
      for(int i=1;i<6;i++){
          for(int j=1;j<4;j++){
              mazeVisitados[i][j]=0;
          }
      }
      solveB(xRobot,yRobot);
    }while(cubosColocados<3);
    for(int i=1;i<6;i++){
      for(int j=1;j<4;j++){
        mazeVisitados[i][j]=0;
      }
    }
    fin=false;
    soli=0;
    leaveB(xRobot,yRobot);
    goTo(xRobot,yRobot);
  }else if(z==4){
    //zona c
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("ZONA C");*/
    //moveTo(3,5,4,5,30);    
    lineFollower();
  //}
    delay(100000);
}
//1s = 16cm
//0.9375s = 15cm
//1.09375s = 17.5
//1.875s = 30cm
