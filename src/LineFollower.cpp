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

#include "LineFollower.h"

LineFollower::LineFollower(
    SensorArray& sensArrRef,
    Gyro& gyroRef,
    uint8_t statusLed1,
    uint8_t statusLed2,
    uint8_t inputButton1,
    uint8_t inputButton2) : quickPID(&input,
                                     &output,
                                     &setPoint,
                                     Kp,
                                     Ki,
                                     Kd,
                                     QuickPID::DIRECT) {
    sensorArray = &sensArrRef;
    gyro = &gyroRef;
    quickPID.SetMode(QuickPID::AUTOMATIC);

    led1Pin = statusLed1;
    led2Pin = statusLed2;
    button1Pin = inputButton1;
    button2Pin = inputButton2;
}

void LineFollower::initialize() {
    sensorArray->initialize();
    gyro->initialize();
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    pinMode(button1Pin, INPUT);
    pinMode(button2Pin, INPUT);
}

void LineFollower::updateButtons() {
    button1 = digitalRead(button1Pin);
    button2 = digitalRead(button2Pin);
}

float LineFollower::calculateInput(bool sensorsDigital[N_OF_SENSORS]) {
    float total = 0.0f;
    uint8_t numberOfActiveSensors = 0;
    for (size_t i = 0; i < N_OF_SENSORS; i++) {
        if (sensorsDigital[i]) {
            total += i;
            numberOfActiveSensors++;
        }
    }
    return total / numberOfActiveSensors;
}

void LineFollower::run() {
    updateButtons();
    if (!gyroWasCalibrated) {
        Serial.println(button1);
        Serial.println(button2);
        if (button1) {
            digitalWrite(led1Pin, HIGH);
            digitalWrite(led2Pin, HIGH);
            delay(3);

            digitalWrite(led2Pin, LOW);
            if (gyro->calibrate()) {
                digitalWrite(led1Pin, LOW);
                gyroWasCalibrated = true;
            }
        }
        return;
    }
    sensorArray->updateSensorsArray();
    input = calculateInput(sensorArray->sensorsDigital);
    quickPID.Compute();
    gyro->update();

    rotationSpeed = gyro->gyroscope.z;

    delay(200);
}