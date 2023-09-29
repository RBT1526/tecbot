//Receiver code
 #include <SoftwareSerial.h>
 SoftwareSerial link(11, 10); // Rx, Tx
  
int reading;

const int pwm_a = 9;
const int der_a = 8;
const int der_b = 7;
const int pwm_b = 3;
const int izq_a = 5;
const int izq_b = 4;
const int standBy = 6;

const int max_vel = 90;

int vel_d = 90;
int vel_i = 90;

float Kp = 0.01;
float Kd = 0.01;
float Ki = 0.01;

float error;
float error_ant;

  void pid_check(int lectura){
    error = 0 - lectura;

    
    vel_i = max_vel + (Kp*error+Kd*(error-error_ant)+Ki*(error+error_ant));
    vel_d = max_vel - (Kp*error+Kd*(error-error_ant)+Ki*(error+error_ant));
    error_ant = error;

    if(vel_d > 255){
      vel_d = 255;
    } 
    if(vel_d<0){
      vel_d = 0;
    }
    if(vel_i > 255){
      vel_i = 255;
    }
    if(vel_i<0){
      vel_i = 0;
    }
    
    //digitalWrite(standBy, HIGH);
    
  }

  void setup() 
  {
    link.begin(9600); //setup software serial
    Serial.begin(9600);    //setup serial monitord
    pinMode(standBy, OUTPUT);
pinMode(pwm_a, OUTPUT);
pinMode(der_a, OUTPUT);
pinMode(der_b, OUTPUT);
pinMode(pwm_b, OUTPUT);
pinMode(izq_a, OUTPUT);
pinMode(izq_b, OUTPUT);
digitalWrite(standBy, LOW);
digitalWrite(izq_a,HIGH);
  digitalWrite(izq_b,LOW);
  digitalWrite(der_a,HIGH);
  digitalWrite(der_b,LOW);
  }

  void loop()  
  {  
  link.listen();
  while (link.available() < 1) {}
  if (link.available() > 1) 
  {
    Serial.println("Data from port two:");
    byte h = link.read();       
    byte l = link.read();
    reading = (h << 8) + l;
    pid_check(reading-2500);
    
    Serial.print("Reading = ");
    Serial.print(reading-2500);
    Serial.print("  vel_d = ");
    Serial.print(vel_d);
    Serial.print("  vel_i = ");
    Serial.println(vel_i);
    

    analogWrite(pwm_a,vel_d);//front
    analogWrite(pwm_b,vel_i);
  
  }
   
  }