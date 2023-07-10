// Copyright 2023 Rafael Farias
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "SensorArray.h"

SensorArray::SensorArray(uint8_t multiplexerIOPin,
                         uint8_t multiplexerS0Pin,
                         uint8_t multiplexerS1Pin,
                         uint8_t multiplexerS2Pin,
                         uint8_t ledSelector1Pin,
                         uint8_t ledSelector2Pin) {
    mplxIOPin = multiplexerIOPin;
    mplxS0Pin = multiplexerS0Pin;
    mplxS1Pin = multiplexerS1Pin;
    mplxS2Pin = multiplexerS2Pin;
    ledSelec1Pin = ledSelector1Pin;
    ledSelec2Pin = ledSelector2Pin;

    pinMode(mplxIOPin, INPUT);
    pinMode(mplxS0Pin, OUTPUT);
    pinMode(mplxS1Pin, OUTPUT);
    pinMode(mplxS2Pin, OUTPUT);
    pinMode(ledSelec1Pin, OUTPUT);
    pinMode(ledSelec2Pin, OUTPUT);

    for (int i = 0; i < N_OF_SENSORS; i++) {
        minRead[i] = UINT16_MAX;
        maxRead[i] = 0;
    }
}

void SensorArray::calibrateSensors() {
    updateSensorsArray();
    for (int i = 0; i < N_OF_SENSORS; i++) {
        if (sensorsAnalog[i] > maxRead[i]) {
            maxRead[i] = sensorsAnalog[i];
        }
        if (sensorsAnalog[i] < minRead[i]) {
            minRead[i] = sensorsAnalog[i];
        }
        sensorsThreshold[i] = (maxRead[i] + minRead[i]) / 2;
    }
}

void SensorArray::printAllAnalog() {
#ifdef SERIAL_DEBUG
    for (uint16_t i = 0; i < N_OF_SENSORS; i++) {
        Serial.print(sensorsAnalog[i]);
        Serial.print(",");
    }
    Serial.println(".");
#endif
}

void SensorArray::printAllDigital() {
#ifdef SERIAL_DEBUG
    for (uint16_t i = 0; i < N_OF_SENSORS; i++) {
        Serial.print(sensorsDigital[i]);
        Serial.print(",");
    }
    Serial.println(".");
#endif
}

void SensorArray::selectSensor(uint8_t sensorIndex) {
    /*
        S0  S1  S2  Channel
        L   L   L   Y0 = S7
        L   L   H   Y1 = S6
        L   H   L   Y2 = S5
        L   H   H   Y3 = S8
        H   L   L   Y4 = S4
        H   L   H   Y5 = S1
        H   H   L   Y6 = S3
        H   H   H   Y7 = S2
    */

    switch (sensorIndex) {
        case 6:
            digitalWrite(mplxS0Pin, LOW);
            digitalWrite(mplxS1Pin, LOW);
            digitalWrite(mplxS2Pin, LOW);
            break;
        case 5:
            digitalWrite(mplxS0Pin, LOW);
            digitalWrite(mplxS1Pin, LOW);
            digitalWrite(mplxS2Pin, HIGH);
            break;
        case 4:
            digitalWrite(mplxS0Pin, LOW);
            digitalWrite(mplxS1Pin, HIGH);
            digitalWrite(mplxS2Pin, LOW);
            break;
        case 7:
            digitalWrite(mplxS0Pin, LOW);
            digitalWrite(mplxS1Pin, HIGH);
            digitalWrite(mplxS2Pin, HIGH);
            break;
        case 3:
            digitalWrite(mplxS0Pin, HIGH);
            digitalWrite(mplxS1Pin, LOW);
            digitalWrite(mplxS2Pin, LOW);
            break;
        case 0:
            digitalWrite(mplxS0Pin, HIGH);
            digitalWrite(mplxS1Pin, LOW);
            digitalWrite(mplxS2Pin, HIGH);
            break;
        case 2:
            digitalWrite(mplxS0Pin, HIGH);
            digitalWrite(mplxS1Pin, HIGH);
            digitalWrite(mplxS2Pin, LOW);
            break;
        case 1:
            digitalWrite(mplxS0Pin, HIGH);
            digitalWrite(mplxS1Pin, HIGH);
            digitalWrite(mplxS2Pin, HIGH);
            break;

        default:
            digitalWrite(mplxS0Pin, LOW);
            digitalWrite(mplxS1Pin, LOW);
            digitalWrite(mplxS2Pin, LOW);
            break;
    }
}

uint16_t SensorArray::analogReadSensorAt(uint8_t sensorIndex) {
    selectSensor(sensorIndex);
    return analogRead(mplxIOPin);
}

uint16_t SensorArray::digitalReadSensorAt(uint8_t sensorIndex) {
    selectSensor(sensorIndex);
    return digitalRead(mplxIOPin);
}

void SensorArray::updateSensorsArray() {
#ifndef LED_ALWAYS_ON
    // Sets the P-channel MOSFFET gate to LOW, turning it on
    digitalWrite(ledSelec1Pin, LOW);

    // Sets the P-channel MOSFFET gate to HIGH, turning it off
    digitalWrite(ledSelec2Pin, HIGH);

#endif

#ifdef DIGITAL_READINGS
    sensorsDigital[0] = digitalReadSensorAt(0);

    sensorsDigital[2] = digitalReadSensorAt(2);

    sensorsDigital[4] = digitalReadSensorAt(4);

    sensorsDigital[6] = digitalReadSensorAt(6);

#else
    sensorsAnalog[0] = analogReadSensorAt(0);

    sensorsAnalog[2] = analogReadSensorAt(2);

    sensorsAnalog[4] = analogReadSensorAt(4);

    sensorsAnalog[6] = analogReadSensorAt(6);

#endif

#ifndef LED_ALWAYS_ON
    digitalWrite(ledSelec1Pin, HIGH);

    digitalWrite(ledSelec2Pin, LOW);

#endif

#ifdef DIGITAL_READINGS
    sensorsDigital[1] = digitalReadSensorAt(1);

    sensorsDigital[3] = digitalReadSensorAt(3);

    sensorsDigital[5] = digitalReadSensorAt(5);

    sensorsDigital[7] = digitalReadSensorAt(7);
#else
    sensorsAnalog[1] = analogReadSensorAt(1);

    sensorsAnalog[3] = analogReadSensorAt(3);

    sensorsAnalog[5] = analogReadSensorAt(5);

    sensorsAnalog[7] = analogReadSensorAt(7);

    updateDigitalValueOfSensors();
#endif
}

void SensorArray::updateDigitalValueOfSensors() {
    for (int i = 0; i < N_OF_SENSORS; i++) {
        if (sensorsAnalog[i] > sensorsThreshold[i]) {
            sensorsDigital[i] = true;
        } else {
            sensorsDigital[i] = false;
        }
    }
}