void setup(){
    Serial.begin(115200);
}
void loop(){
    float adc = analogRead(A0);
    float cm = pow(3027.4 / adc, 1.2134);
    Serial.println(cm);
    delay(10);
}