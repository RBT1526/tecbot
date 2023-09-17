const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;


void setup (){
pinMode(pwm_a, OUTPUT);
pinMode(der_a, OUTPUT);
pinMode(der_b, OUTPUT);
Serial.begin(115200);

}
void loop(){
analogWrite(pwm_a,150);
digitalWrite(der_a,HIGH);
digitalWrite(der_b,LOW);
delay(2000);
analogWrite(pwm_a,150);
digitalWrite(der_a,LOW);
digitalWrite(der_b,HIGH);
delay(2000);
}