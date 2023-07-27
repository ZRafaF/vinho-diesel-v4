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
                         uint8_t ledSelector2Pin,
                         uint8_t leftHelperPin,
                         uint8_t rightHelperPin,
                         LineColor colorOfTheLine,
                         bool useAnalogSensors) {
    _mplxIOPin = multiplexerIOPin;
    _mplxS0Pin = multiplexerS0Pin;
    _mplxS1Pin = multiplexerS1Pin;
    _mplxS2Pin = multiplexerS2Pin;
    _ledSelec1Pin = ledSelector1Pin;
    _ledSelec2Pin = ledSelector2Pin;
    _leftHelperPin = leftHelperPin;
    _rightHelperPin = rightHelperPin;
    lineColor = colorOfTheLine;
    readsAnalog = useAnalogSensors;

    for (int i = 0; i < N_OF_SENSORS; i++) {
        minRead[i] = UINT16_MAX;
        maxRead[i] = 0;
    }
}

void SensorArray::initialize() {
    pinMode(_mplxIOPin, INPUT);
    pinMode(_mplxS0Pin, OUTPUT);
    pinMode(_mplxS1Pin, OUTPUT);
    pinMode(_mplxS2Pin, OUTPUT);
    pinMode(_ledSelec1Pin, OUTPUT);
    pinMode(_ledSelec2Pin, OUTPUT);
    pinMode(_leftHelperPin, INPUT);
    pinMode(_rightHelperPin, INPUT);
}

void SensorArray::calibrateSensors() {
    updateSensorsArray();
    for (int i = 0; i < N_OF_SENSORS; i++) {
        if (sensorRaw[i] > maxRead[i]) {
            maxRead[i] = sensorRaw[i];
        }
        if (sensorRaw[i] < minRead[i]) {
            minRead[i] = sensorRaw[i];
        }
        sensorsThreshold[i] = (maxRead[i] + minRead[i]) / 2;
    }
}

void SensorArray::printAllRaw() {
#ifdef SERIAL_DEBUG
    Serial.print(leftSensRaw);
    Serial.print("---");

    for (uint16_t i = 0; i < N_OF_SENSORS; i++) {
        Serial.print(sensorRaw[i]);
        Serial.print(",");
    }
    Serial.print("---");
    Serial.println(rightSensRaw);
#endif
}

void SensorArray::printAllProcessed() {
#ifdef SERIAL_DEBUG
    Serial.print(leftSensProcessed);
    Serial.print("---");

    for (uint16_t i = 0; i < N_OF_SENSORS; i++) {
        Serial.print(sensorProcessed[i]);
        Serial.print(",");
    }

    Serial.print("---");
    Serial.println(rightSensProcessed);
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
            digitalWrite(_mplxS0Pin, LOW);
            digitalWrite(_mplxS1Pin, LOW);
            digitalWrite(_mplxS2Pin, LOW);
            break;
        case 5:
            digitalWrite(_mplxS0Pin, LOW);
            digitalWrite(_mplxS1Pin, LOW);
            digitalWrite(_mplxS2Pin, HIGH);
            break;
        case 4:
            digitalWrite(_mplxS0Pin, LOW);
            digitalWrite(_mplxS1Pin, HIGH);
            digitalWrite(_mplxS2Pin, LOW);
            break;
        case 7:
            digitalWrite(_mplxS0Pin, LOW);
            digitalWrite(_mplxS1Pin, HIGH);
            digitalWrite(_mplxS2Pin, HIGH);
            break;
        case 3:
            digitalWrite(_mplxS0Pin, HIGH);
            digitalWrite(_mplxS1Pin, LOW);
            digitalWrite(_mplxS2Pin, LOW);
            break;
        case 0:
            digitalWrite(_mplxS0Pin, HIGH);
            digitalWrite(_mplxS1Pin, LOW);
            digitalWrite(_mplxS2Pin, HIGH);
            break;
        case 2:
            digitalWrite(_mplxS0Pin, HIGH);
            digitalWrite(_mplxS1Pin, HIGH);
            digitalWrite(_mplxS2Pin, LOW);
            break;
        case 1:
            digitalWrite(_mplxS0Pin, HIGH);
            digitalWrite(_mplxS1Pin, HIGH);
            digitalWrite(_mplxS2Pin, HIGH);
            break;

        default:
            digitalWrite(_mplxS0Pin, LOW);
            digitalWrite(_mplxS1Pin, LOW);
            digitalWrite(_mplxS2Pin, LOW);
            break;
    }
}

uint16_t SensorArray::readSensorAt(uint8_t sensorIndex) {
    selectSensor(sensorIndex);
    return readsAnalog ? analogRead(_mplxIOPin) : digitalRead(_mplxIOPin);
}

void SensorArray::updateSensorsArray() {
    leftSensRaw = digitalRead(_leftHelperPin);
    rightSensRaw = digitalRead(_rightHelperPin);

    leftSensProcessed = lineColor == BLACK ? leftSensRaw : !leftSensRaw;
    rightSensProcessed = lineColor == BLACK ? rightSensRaw : !rightSensRaw;

#ifndef LED_ALWAYS_ON
    // Sets the P-channel MOSFFET gate to LOW, turning it on
    digitalWrite(ledSelec1Pin, LOW);

    // Sets the P-channel MOSFFET gate to HIGH, turning it off
    digitalWrite(ledSelec2Pin, HIGH);

#endif

    sensorRaw[0] = readSensorAt(0);

    sensorRaw[2] = readSensorAt(2);

    sensorRaw[4] = readSensorAt(4);

    sensorRaw[6] = readSensorAt(6);

#ifndef LED_ALWAYS_ON
    digitalWrite(ledSelec1Pin, HIGH);

    digitalWrite(ledSelec2Pin, LOW);

#endif

    sensorRaw[1] = readSensorAt(1);

    sensorRaw[3] = readSensorAt(3);

    sensorRaw[5] = readSensorAt(5);

    sensorRaw[7] = readSensorAt(7);

    processReadings();
}

void SensorArray::processReadings() {
    uint8_t lineStartsAt = 0;
    uint8_t lineEndsAt = N_OF_SENSORS - 1;

    for (uint8_t i = 0; i < N_OF_SENSORS; i++) {
        if (!readsAnalog) {
            if (sensorRaw[i]) {
                sensorProcessed[i] = lineColor == BLACK ? true : false;
            } else {
                sensorProcessed[i] = lineColor == BLACK ? false : true;
            }
        } else {
            if (sensorRaw[i] > sensorsThreshold[i]) {
                sensorProcessed[i] = lineColor == BLACK ? true : false;
            } else {
                sensorProcessed[i] = lineColor == BLACK ? false : true;
            }
        }

        if (i > 0) {
            if (sensorProcessed[i] && !sensorProcessed[i - 1]) {
                lineStartsAt = i;
            }
        }

        // Checking for end of line
        if (i < N_OF_SENSORS) {
            if (!sensorProcessed[i] && sensorProcessed[i - 1]) {
                lineEndsAt = i;
            }
        }
    }
    for (uint8_t i = 0; i < N_OF_SENSORS; i++) {
        if (i < lineStartsAt || i > lineEndsAt)
            sensorProcessed[i] = 0;
    }
}