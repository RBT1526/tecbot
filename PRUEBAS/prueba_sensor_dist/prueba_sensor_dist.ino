
void setup(){
    Serial.begin(115200);
}
void loop(){
    int s = analogRead(A0);//izq
    float dist= pow(10,log10(s/1821.2)/-0.65);
    int s1 = analogRead(A1);//der
    float dist1= pow(10,log10(s1/1821.2)/-0.65);
    int s2 = analogRead(A2);//back
    float dist2= pow(10,log10(s2/1821.2)/-0.65);
    int s3 = analogRead(A3);//front
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

//s3 enfrente
//s2 izquierda
//s4 derecha
//s1 atras