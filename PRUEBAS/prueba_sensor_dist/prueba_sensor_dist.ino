void setup(){
    Serial.begin(115200);
}
void loop(){
    int s = analogRead(A0);
    int dist= pow(10,log10(s/1821.2)/-0.65);
    Serial.println(dist);
    delay(100);
}