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
    PIDestal& pidLRef,
    PIDestal& pidRRef,
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
    pidL = &pidLRef;
    pidL->errorTolerance = 1;
    pidR = &pidRRef;
    pidR->errorTolerance = 1;
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
    remotePid->initialize("VINHO_DIESEL", "Diesel");
#endif

    sensorArray->initialize();
    gyro->initialize();

    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    pinMode(button1Pin, INPUT);
    pinMode(button2Pin, INPUT);

    motors->begin();
}

void LineFollower::updateButtons() {
    button1 = digitalRead(button1Pin);
    button2 = digitalRead(button2Pin);

    if (button1) {
        toggleMotorsAreActive();
    }
}

void LineFollower::toggleMotorsAreActive() {
    unsigned long currentTime = millis();
    if (currentTime > lastPressedButtonTime + 500) {
        lastPressedButtonTime = currentTime;
        motorsAreActive = !motorsAreActive;
    }
}

float LineFollower::calculateInput(bool sensorsProcessed[N_OF_SENSORS]) {
    float total = 0.0f;
    uint8_t numberOfActiveSensors = 0;
    for (size_t i = 0; i < N_OF_SENSORS; i++) {
        if (sensorsProcessed[i]) {
            total += i;
            numberOfActiveSensors++;
        }
    }
    if (numberOfActiveSensors) {
        const float inputResult = total / numberOfActiveSensors;
        lastValidSensorInput = inputResult;
    }
    return lastValidSensorInput;
}

float LineFollower::calculateTargetRotSpeed(float error) {
    float absError = abs(error);
    float errorSignal = error / absError;
    if (absError > 0 && absError <= 1) return errorSignal * 10;
    if (absError > 1 && absError <= 2) return errorSignal * 20;
    if (absError > 2 && absError <= 3) return errorSignal * 30;
    if (absError > 3 && absError <= 4) return errorSignal * 40;
    if (absError > 4) return errorSignal * 50;

    return (error * error) * 2;
}

void LineFollower::updateMotors() {
    leftMotorOutput = motorOffset + (pidResult * errorGain);
    rightMotorOutput = motorOffset - (pidResult * errorGain);

    if (leftMotorOutput > motorClamp) leftMotorOutput = motorClamp;
    if (leftMotorOutput < -motorClamp) leftMotorOutput = -motorClamp;
    if (rightMotorOutput > motorClamp) rightMotorOutput = motorClamp;
    if (rightMotorOutput < -motorClamp) rightMotorOutput = -motorClamp;

    motors->drive(leftMotorOutput, rightMotorOutput);
}

void LineFollower::printAll() {
#ifdef SERIAL_DEBUG
    Serial.print("input: ");
    Serial.print(sensorInput);
    Serial.print("\t");
    Serial.print("pidR: ");
    Serial.print(pidResult);
    Serial.print("\t");
    Serial.print("targetR: ");
    Serial.print(rotSpeedTarget);
    Serial.print("\t");
    Serial.print("rotSpeed: ");
    Serial.print(rotSpeed);
    Serial.print("\t");
    Serial.print("ML: ");
    Serial.print(leftMotorOutput);
    Serial.print("\t");
    Serial.print("MR: ");
    Serial.print(rightMotorOutput);
    Serial.print("\t");
    Serial.print("pL: ");
    Serial.print(pidL->getPidConsts().p);
    Serial.print(" pR: ");
    Serial.print(pidR->getPidConsts().p);
    Serial.print("\t");
    Serial.print(remotePid->getExtraInfo());

    Serial.println();

#endif
}

void LineFollower::run() {
#ifdef USE_BLUETOOTH
    remotePid->process();
    if (remotePid->getExtraInfo()[0] == "a"[0]) {
        toggleMotorsAreActive();
        remotePid->setExtraInfo("b");
    }
#endif
    digitalWrite(led2Pin, motorsAreActive ? HIGH : LOW);
    updateButtons();
    if (!gyroWasCalibrated) {
        digitalWrite(led1Pin, HIGH);
        digitalWrite(led2Pin, HIGH);
        if (gyro->calibrate()) {
            digitalWrite(led1Pin, LOW);
            digitalWrite(led2Pin, LOW);
            gyroWasCalibrated = true;
        }
    }
    sensorArray->updateSensorsArray();
    sensorInput = calculateInput(sensorArray->sensorProcessed);
    gyro->update();

    rotSpeedTarget = calculateTargetRotSpeed(sensorTarget - sensorInput);
    rotSpeed = gyro->rotationSpeed;

    if (motorsAreActive) {
        pidResult += pidL->calculate(rotSpeed - rotSpeedTarget);

        updateMotors();
    } else {
        pidResult = 0;
        motors->coast();
    }
    printAll();

    delay(20);
}