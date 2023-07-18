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
    PIDestal& pidRef,
    Tb6612fng& motorsRef,
#ifdef USE_BLUETOOTH
    PIDestalRemoteBLE& remotePidRef,
#endif
    uint8_t statusLed1,
    uint8_t statusLed2,
    uint8_t inputButton1,
    uint8_t inputButton2) {
    sensorArray = &sensArrRef;
    gyro = &gyroRef;
    pid = &pidRef;
    motors = &motorsRef;
#ifdef USE_BLUETOOTH
    remotePid = &remotePidRef;
#endif

    led1Pin = statusLed1;
    led2Pin = statusLed2;
    button1Pin = inputButton1;
    button2Pin = inputButton2;
}

void LineFollower::initialize() {
#ifdef USE_BLUETOOTH
    remotePid->initialize("VINHO_DIESEL", "diesel");
#endif

    sensorArray->initialize();
    gyro->initialize();

    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    pinMode(button1Pin, INPUT);
    pinMode(button2Pin, INPUT);
    digitalWrite(led2Pin, HIGH);

    motors->begin();
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

float LineFollower::calculateTargetRotSpeed(float error) {
    return error * 1000;
}

void LineFollower::printAll() {
#ifdef SERIAL_DEBUG
    Serial.print("input: ");
    Serial.print(sensorInput);
    Serial.print("\t");
    Serial.print("pidResult: ");
    Serial.print(pidResult);
    Serial.print("\t");
    Serial.print("rotSpeed: ");
    Serial.print(rotSpeed);
    Serial.print("\t");
    Serial.print("b2: ");
    Serial.print(button1);
    Serial.print("b2: ");
    Serial.println(button2);

#endif
}

void LineFollower::run() {
#ifdef USE_BLUETOOTH
    remotePid->process();
    remotePid->setExtraInfo("t: " + String(rotSpeedTarget) + " r: " + String(rotSpeed));
#endif
    updateButtons();
    if (!gyroWasCalibrated) {
        Serial.println(button1);
        Serial.println(button2);
        if (button1) {
            digitalWrite(led1Pin, HIGH);
            delay(1000);

            digitalWrite(led2Pin, LOW);
            if (gyro->calibrate()) {
                digitalWrite(led1Pin, LOW);
                gyroWasCalibrated = true;
            }
        }
        delay(200);
        return;
    }
    sensorArray->updateSensorsArray();
    sensorInput = calculateInput(sensorArray->sensorsDigital);
    gyro->update();

    rotSpeedTarget = calculateTargetRotSpeed(sensorTarget - sensorInput);
    rotSpeed = gyro->gyroscope.z;

    pidResult = pid->calculate(rotSpeed - rotSpeedTarget);

    printAll();

    delay(20);
}