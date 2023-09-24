/*

 * Copyright (c) 2016 Intel Corporation.  All rights reserved.

 * See the bottom of this file for the license terms.

 */

#include <CurieBLE.h>

BLEPeripheral blePeripheral;  // Bluetooth® Low Energy Peripheral Device (the board you're programming)

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


const int pwm_a = 3;
const int der_a = 4;
const int der_b = 5;
const int pwm_b = 9;
const int izq_a = 8;
const int izq_b = 7;
const int standBy = 6;

int velocity = 130;
void setup() {

  Serial.begin(115200);
  pinMode(standBy, OUTPUT);
pinMode(pwm_a, OUTPUT);
pinMode(der_a, OUTPUT);
pinMode(der_b, OUTPUT);
pinMode(pwm_b, OUTPUT);
pinMode(izq_a, OUTPUT);
pinMode(izq_b, OUTPUT);
digitalWrite(standBy, HIGH);

  blePeripheral.setLocalName("LED");

  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());


  blePeripheral.addAttribute(ledService);

  blePeripheral.addAttribute(switchCharacteristic);


  switchCharacteristic.setValue(130);

  blePeripheral.begin();

  Serial.println("BLE LED Peripheral");
}

void loop() {

  // listen for Bluetooth® Low Energy peripherals to connect:

  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  

  if (central) {

    Serial.print("Connected to central: ");

    Serial.println(central.address());

    while (central.connected()) {
        analogWrite(pwm_a,150);//front
        digitalWrite(der_a,HIGH);
        digitalWrite(der_b,LOW);
        analogWrite(pwm_b,velocity);
        digitalWrite(izq_a,HIGH);
        digitalWrite(izq_b,LOW);

      if (switchCharacteristic.written()) {

        velocity = switchCharacteristic.value();
      }

    }

    Serial.print(F("Disconnected from central: "));

    Serial.println(central.address());

  }
}
