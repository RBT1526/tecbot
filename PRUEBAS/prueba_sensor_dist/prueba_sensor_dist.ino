
void setup(){
    Serial.begin(115200);
}
void loop(){
    int s = analogRead(A0);
    float dist= pow(10,log10(s/1821.2)/-0.65);
    int s1 = analogRead(A1);
    float dist1= pow(10,log10(s1/1821.2)/-0.65);
    int s2 = analogRead(A2);
    float dist2= pow(10,log10(s2/1821.2)/-0.65);
    int s3 = analogRead(A3);
    float dist3= pow(10,log10(s3/1821.2)/-0.65);
    Serial.print("s1: ");
    Serial.print(dist);
   Serial.print(" s2: ");
    Serial.print(dist1);
    Serial.print(" s3: ");
    Serial.print(dist2);
    Serial.print(" s4: ");
    Serial.println(dist3);
    delay(100);
}