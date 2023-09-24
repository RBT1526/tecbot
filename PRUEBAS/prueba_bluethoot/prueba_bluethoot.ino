/*

 * Copyright (c) 2016 Intel Corporation.  All rights reserved.

 * See the bottom of this file for the license terms.

 */

#include <CurieBLE.h>

BLEPeripheral blePeripheral;  // Bluetooth® Low Energy Peripheral Device (the board you're programming)

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = 13; // pin to use for the LED

void setup() {

  Serial.begin(9600);


  pinMode(ledPin, OUTPUT);

  blePeripheral.setLocalName("LED");

  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());


  blePeripheral.addAttribute(ledService);

  blePeripheral.addAttribute(switchCharacteristic);


  switchCharacteristic.setValue(0);

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

      if (switchCharacteristic.written()) {

        if (switchCharacteristic.value()) {  

          Serial.println("LED on");

          digitalWrite(ledPin, HIGH);         // will turn the LED on

        } else {                              // a 0 value

          Serial.println(F("LED off"));

          digitalWrite(ledPin, LOW);          // will turn the LED off

        }

      }

    }

    Serial.print(F("Disconnected from central: "));

    Serial.println(central.address());

  }
}

/*

   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or

   modify it under the terms of the GNU Lesser General Public

   License as published by the Free Software Foundation; either

   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,

   but WITHOUT ANY WARRANTY; without even the implied warranty of

   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU

   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public

   License along with this library; if not, write to the Free Software

   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/