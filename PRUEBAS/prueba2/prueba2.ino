void setup(){
    Serial.begin(115200);

}

void loop(){
    Serial.println(analogRead(A3));
    delay(100);
}