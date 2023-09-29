/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-rotary-encoder
 */

//encoder stuff
#include <ezButton.h>
#define CLK_PIN 2//change
#define DT_PIN 3
#define SW_PIN 4
#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction
int counter = 0;
int direction = DIRECTION_CW;
int CLK_state;
int prev_CLK_state;
ezButton button(SW_PIN);  // create ezButton object that attach to pin 4

//motores
const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 7;
const int izq_b = 8;
const int standBy = 6;
void motor_drive(int v1,int v2){
    analogWrite(pwm_a,v1);
    digitalWrite(der_a,HIGH);
    digitalWrite(der_b,LOW);
    analogWrite(pwm_b,v2);
    digitalWrite(izq_a,LOW);
    digitalWrite(izq_b,HIGH);
}

void setup() {
  Serial.begin(9600);

  pinMode(standBy, OUTPUT);
  pinMode(pwm_a, OUTPUT);
  pinMode(der_a, OUTPUT);
  pinMode(der_b, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(izq_a, OUTPUT);
  pinMode(izq_b, OUTPUT);
  digitalWrite(standBy, LOW);

  //setup encoder
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  button.setDebounceTime(50);
  prev_CLK_state = digitalRead(CLK_PIN);
}

void moveFor(int d){
  counter=0;
  int targetTicks=d/1.149;
  while(counter<=targetTicks){ //tick = 1.149cm , d/1.149 = ticks a avanzar
    motor_drive(100,100);//AVANZAR
    button.loop();
    CLK_state = digitalRead(CLK_PIN);
      if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
        if (digitalRead(DT_PIN) == HIGH) { //izq
          counter--;
          direction = DIRECTION_CCW;
        } else { //der
          counter++;
          direction = DIRECTION_CW;
        }
      prev_CLK_state = CLK_state;
    }
  }
}

void loop() {
  while(counter<19){ //tick = 1.149cm , d/1.149 = ticks a avanzar
    motor_drive(100,100);
    button.loop();  // MUST call the loop() function first
    // read the current state of the rotary encoder's CLK pin
    CLK_state = digitalRead(CLK_PIN);
    // If the state of CLK is changed, then pulse occurred
    // React to only the rising edge (from LOW to HIGH) to avoid double count
    if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
      // if the DT state is HIGH
      // the encoder is rotating in counter-clockwise direction => decrease the counter
      if (digitalRead(DT_PIN) == HIGH) {
        counter--;
        direction = DIRECTION_CCW;
      } else {
        // the encoder is rotating in clockwise direction => increase the counter
        counter++;
        direction = DIRECTION_CW;
      }

      Serial.print("DIRECTION: ");
      if (direction == DIRECTION_CW)
        Serial.print("Clockwise");
      else
        Serial.print("Counter-clockwise");

      Serial.print(" | COUNTER: ");
      Serial.println(counter);
    }

    // save last CLK state
    prev_CLK_state = CLK_state;

    if (button.isPressed()) {
      Serial.println("The button is pressed");
    }
  }
}
