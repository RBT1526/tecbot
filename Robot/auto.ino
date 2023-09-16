#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

int path[20][2];
int soli=0;
int colores[3]={0,0,0}; // 0 azul 1 rojo 2 verde
int targetColor=2;
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
//orientacion del robot (1, 2, 3, 4)
int orientacion=1;
//posicion del robot
int xRobot=2;
int yRobot=5;
//moverme hacia coordenada
void moveTo(int x1,int y1,int x2,int y2){
    //CHECAR Q NO SEA NEGRO MIENTRAS ME MUEVO
    if(x1==x2){
        //mover en y
        if(y1>y2){

        }else{

        }
    }else{
        //mover en x
        if(x1>x2){

        }else{

        }
    }
    xRobot=x2;
    yRobot=y2;
    return;
}
// recorrer path
void goTo(int x1,int y1){ //parametros = posicion inicial
    for(int i=0;i<soli;i++){
        moveTo(x1,y1,path[i][0],path[i][1]);
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
        moveTo(x1,y1,path[i][0],path[i][1]);
        x1=path[i][0];
        y1=path[i][1];

    }
    xRobot=path[0][0];
    yRobot=path[0][1];
    return;
}

//función recursiva para checar cada cuadro posible
void scanMaze(int x,int y){
    for(int i=0;i<4;i++){ //CHECAR MUROS
        if(mazeParedes[i][y][x]==0){
            //1 si hay pared
            //2 si no hay
            mazeParedes[i][y][x]=2;
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
        moveTo(xRobot,yRobot,x2,y2);
        //CHECAR Y GUARDAR COLOR
        mazeColores[y][x]=2;
      scanMaze(x2,y2);
        path[soli][0]=x;
        path[soli][1]=y;
        soli++;
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