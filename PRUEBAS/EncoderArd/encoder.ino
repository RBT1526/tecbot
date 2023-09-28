int encoder0PinA = 3;
int encoder0PinB = 4;
int motor = 10;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;

void setup() {
  Serial.begin (9600);
  pinMode (encoder0PinA, INPUT_PULLUP);
  pinMode (encoder0PinB, INPUT_PULLUP);
  pinMode (motor, OUTPUT);
  
}

void loop() {
  while (encoder0Pos < 10){
    encoder();
    digitalWrite(motor, HIGH);
  }
  digitalWrite(motor, LOW);
  
}

void encoder() {
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
    Serial.println (encoder0Pos);
   
  }
  encoder0PinALast = n;
}