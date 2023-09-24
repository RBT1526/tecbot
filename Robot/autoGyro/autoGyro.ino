#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
//IMU
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//cosas del sensor de color
#define commonAnode true
byte gammatable[256];
#define redpin 3
#define greenpin 5
#define bluepin 6
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int path[20][2];
int soli=0;
int colores[3]={0,0,0}; // 0 azul 1 rojo 2 verde
int targetColor=2;

int leerColor(){
  //rutina para leer color
  int IR = analogRead(A3);
  float red, green, blue;  
  tcs.setInterrupt(false);  // turn on LED
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  lcd.clear();
  lcd.setCursor(0,0);  
  if(green>100 && IR>500){
    lcd.print("NEGRO");
    return 5;
  }else if(red>100){
    colores[1]+=1;
    lcd.print("ROJO");
    return 3;    
  }else if(green>100){
    colores[2]+=1;
    lcd.print("VERDE");
    return 4;
  }else if(blue>100){
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
const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;
float velDer=45,velIzq=45;
//cosas para calcular angulos
float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
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
    //roll forward
    do{
    //int front = analogRead(A3);
    //int distFront= pow(10,log10(front/1821.2)/-0.65);
    }while(distFront>4.3);
    //stop
    if(b!=-1){
        orientar(b);
        //roll forward
        do{
        //int front = analogRead(A3);
        //int distFront= pow(10,log10(front/1821.2)/-0.65);
        }while(distFront>4.3);
        //stop
    }
    orientar(oInicial);
    return;
}
//moverme hacia coordenada
void moveTo(int x1,int y1,int x2,int y2,int d){
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
    //AVANZAR d
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
            int s = analogRead(A0);
            int dist= pow(10,log10(s/1821.2)/-0.65);
            if(dist<20){
                //1 si hay pared
                mazeParedes[i][y][x]=1;
            }else{
                //2 si no hay
                mazeParedes[i][y][x]=2;
            }
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
    //roll forward
    do{
        //int der = analogRead(A0);
        //int izq = analogRead(A1);
        //front = analogRead(A3);
        //int distDer= pow(10,log10(der/1821.2)/-0.65);
        //int distIzq= pow(10,log10(izq/1821.2)/-0.65);
        //distFront= pow(10,log10(front/1821.2)/-0.65);
        if(distDer<=4){
            //subir velocidad motor der
        }else if(distIzq<=4){
            //subir velocidad motor izq
        }
    }while(distFront>4.3);
    //stop
    center(0,270);
    orientar(180);
    return;
}
int cubosColocados=0;
bool tengoCubo=false;
int coloresB(){
    //rutina para leer color
  int IR = analogRead(A3);
  float red, green, blue;  
  tcs.setInterrupt(false);  // turn on LED
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  if(red>100){
    return 1;    
  }else if(green>100){
    return 2;
  }
  return 3;
}
void solveB(int x,int y){
    //1 rojo
    //2 verde
    //3 vacio
    for(int i=0;i<4;i++){ //moverme a cada direccion
    int x2=x+dx[i];
    int y2=y+dy[i];
    if(mazeColores[y2][x2]!=-1 && mazeVisitados[y2][x2]==0 && cubosColocados<3){
        mazeVisitados[y2][x2]=1;
        moveTo(xRobot,yRobot,x2,y2,15); //moverme 15 y checar color
        mazeColores[y2][x2]=coloresB();
        if(mazeColores[y2][x2]==1 && tengoCubo==false){
            //lo agarro
            tengoCubo=true;
            moveTo(xRobot,yRobot,x2,y2,15);
            fin=false;
            findGreen();
            goTo();
        }else if(mazeColores[y2][x2]==2 && tengoCubo==true){
            mazeVisitados[y2][x2]=1;
            //lo dejo
            moveTo(xRobot,yRobot,x2,y2,15);
        }else if(mazeColores[y2][x2]==3){
            moveTo(xRobot,yRobot,x2,y2,15);
            solveB(x2,y2);
            moveTo(xRobot,yRobot,x,y,-30);
        }
    }
  }
    return;
}
void setup(){
    Serial.begin(115200);
//cosas del IMU
    CurieIMU.begin();
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 0);
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setGyroRange(250);
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
//cosas de la pantalla
  tcs.begin();
  lcd.init();
  lcd.backlight();

  #if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
  //rutina zona a
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
  //rutina rampa
  downRamp();
  //rutina zona b
  for(int i=1;i<6;i++){
    for(int j=1;j<4;j++){
        mazeColores[i][j]=0;
        mazeVisitados[i][j]=0;
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
  }while(cubosColocados<3)
}