const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;

void setup (){
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
void loop(){
analogWrite(pwm_a,150);//front
digitalWrite(der_a,HIGH);
digitalWrite(der_b,LOW);
analogWrite(pwm_b,136);
digitalWrite(izq_a,HIGH);
digitalWrite(izq_b,LOW);
//delay(2000);
/*analogWrite(pwm_a,150);//left
digitalWrite(der_a,HIGH);
digitalWrite(der_b,LOW);
analogWrite(pwm_b,150);
digitalWrite(izq_a,LOW);
digitalWrite(izq_b,HIGH);
delay(2000);
analogWrite(pwm_a,150);//back
digitalWrite(der_a,LOW);
digitalWrite(der_b,HIGH);
analogWrite(pwm_b,150);
digitalWrite(izq_a,LOW);
digitalWrite(izq_b,HIGH);
delay(2000);
analogWrite(pwm_a,150);//right
digitalWrite(der_a,LOW);
digitalWrite(der_b,HIGH);
analogWrite(pwm_b,150);
digitalWrite(izq_a,HIGH);
digitalWrite(izq_b,LOW);
delay(2000);*/
/*
delay(2000);
analogWrite(pwm_a,150);
digitalWrite(der_a,LOW);
digitalWrite(der_b,HIGH);
delay(2000);
*/
}