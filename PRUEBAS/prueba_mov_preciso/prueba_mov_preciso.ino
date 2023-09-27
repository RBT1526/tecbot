const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
void avanzar(int d){
//AVANZAR d
    int targetDist = analogRead(A2);//CAMBIAR
    int back=targetDist;
    targetDist+=d;
    digitalWrite(der_a,LOW);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,0);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,LOW);
    while(back<targetDist){
        back = analogRead(A2);//CAMBIAR
        Serial.print(back+" "+targetDist);
    }
    analogWrite(pwm_a,0);
    digitalWrite(der_a,LOW);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,0);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,LOW);
}
void setup(){
    Serial.begin(115200);
}
void loop(){
    avanzar(15);
}