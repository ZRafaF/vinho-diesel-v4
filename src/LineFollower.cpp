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

float invertedMap(float input, float inMin, float inMax, float outMin, float outMax) {
    // Invert the input value
    float invertedValue = inMax + inMin - input;

    // Map the inverted value to the output range
    return outMin + (outMax - outMin) * (invertedValue - inMin) / (inMax - inMin);
}

LineFollower::LineFollower(
    SensorArray& sensArrRef,
    Gyro& gyroRef,
    PIDestal& sensorPidRef,
    PIDestal& gyroPidRef,
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
    gyroPid = &gyroPidRef;
    gyroPid->errorTolerance = 0;
    sensorPid = &sensorPidRef;
    sensorPid->errorTolerance = 0;
    sensorPid->setUseDeltaTime(false);
    gyroPid->setUseDeltaTime(false);

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
    if (currentTime > lastPressedButtonTime + 200) {
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
        const float inputResult = total / float(numberOfActiveSensors);
        lastValidSensorInput = inputResult;
    }
    isOutOfLine = numberOfActiveSensors == 0 ? true : false;
    if (numberOfActiveSensors >= 4 && motorsAreActive) {
        lastCrossingTime = millis();
    }

    return lastValidSensorInput;
}

float LineFollower::calculateTargetRotSpeed(float error) {
    const float absError = abs(error);
    const float signalError = error / absError;
    currentController = absError > 3
                            ? GYRO
                            : SENSOR;
    if (error == 0) return 0;

    return (error * 40);
}

void LineFollower::updateMotors() {
    if (currentController == GYRO || isOutOfLine) {
        pidResult = gyroPidResult * errorGain;

    } else {
        pidResult = sensorPidResult * 0.1;
    }
    leftMotorOutput = motorOffset - pidResult;
    rightMotorOutput = motorOffset + pidResult;

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
    Serial.print("SpidR: ");
    Serial.print(sensorPidResult);
    Serial.print("\t");
    Serial.print("GpidR: ");
    Serial.print(gyroPidResult);
    Serial.print("\t");
    Serial.print("targetR: ");
    Serial.print(rotSpeedTarget);
    Serial.print("\t");
    Serial.print("rotSpeed: ");
    Serial.print(rotSpeed);
    Serial.print("\t");
    Serial.print("MOfst: ");
    Serial.print(motorOffset);
    Serial.print("\t");
    Serial.print("ML: ");
    Serial.print(leftMotorOutput);
    Serial.print("\t");
    Serial.print("MR: ");
    Serial.print(rightMotorOutput);
    Serial.print("SensorController?: ");
    Serial.print(currentController == SENSOR ? "SENSOR" : "GYRO");
    Serial.println();

#endif
}

float LineFollower::calculateSensorReadingError(float error) {
    if (error == 0) return 0;
    float absError = abs(error);
    float errorSignal = error / absError;

    return error;
}

float LineFollower::calculateMotorOffset() {
    const float minMapRotSpeed = 5.0;
    const float maxMapRotSpeed = 90.0;
    const float constrainedRot = constrain(abs(rotSpeed), minMapRotSpeed, maxMapRotSpeed);

    return invertedMap(constrainedRot, minMapRotSpeed, maxMapRotSpeed, minMotorOffset, maxMotorOffset);
}

void LineFollower::triggeredInterrupt(HelperSensorSide sensorSide) {
    if (!motorsAreActive || sensorSide == LEFT) return;

    const unsigned int timeNow = millis();
    if (timeNow - lastCrossingTime >= crossingTimeThreshold) {
        numberOfRightSignals++;
        if (numberOfRightSignals >= totalRightSignals) {
            motorsAreActive = false;
        }
    }
}

void LineFollower::changeMode(Modes newMode) {
    if (newMode == SLOW) {
        minMotorOffset = 0.4;
        maxMotorOffset = 0.7;
    }
    if (newMode == MEDIUM) {
        minMotorOffset = 0.6;
        maxMotorOffset = 0.9;
    }
    if (newMode == FAST) {
        minMotorOffset = 0.7;
        maxMotorOffset = 1.0;
    }

#ifdef USE_BLUETOOTH
    if (newMode == SLOW) remotePid->setExtraInfo("SLOW");
    if (newMode == MEDIUM) remotePid->setExtraInfo("MEDIUM");
    if (newMode == FAST) remotePid->setExtraInfo("FAST");

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

    motorOffset = calculateMotorOffset();

    if (motorsAreActive) {
        sensorPidResult = sensorPid->calculate(calculateSensorReadingError(sensorTarget - sensorInput));
        gyroPidResult = gyroPid->calculate(rotSpeedTarget - rotSpeed);
        updateMotors();
    } else {
        gyroPidResult = 0;
        sensorPidResult = 0;
        motors->coast();
    }

    // printAll();
    delay(10);
}