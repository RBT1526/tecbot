char Mymessage[5] = {'H','e','l','l','o'}; //String data
 
  void setup() {
 
    // Begin the Serial at 9600 Baud
 
    Serial.begin(9600);
 
  }
 
 
 
 
  void loop() {
 
    Serial.write(Mymessage,5); //Write the serial data
 
    delay(1000);
 
  }