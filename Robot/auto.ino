#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"
#include "CurieIMU.h"

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
void orientar(int o){
    if(orientacion==0 && o==270){
        //girar izquierda
        orientacion=270;
    }else if(orientacion==270 && o==0){
        //girar derecha
        orientacion=0;
    }else if(orientacion<o){
        //girar derecha
        orientacion+=90;
        orientar(o);
    }else{
        //girar izquierda
        orientacion-=90;
        orientar(o);
    }
    return;
}
//moverme hacia coordenada
void moveTo(int x1,int y1,int x2,int y2,int d){
    //CHECAR Q NO SEA NEGRO MIENTRAS ME MUEVO
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
void setup(){
    Serial.begin(115200);
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
}