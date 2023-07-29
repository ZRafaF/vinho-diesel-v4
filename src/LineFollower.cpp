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

void LineFollower::endRun() {
    shouldStop = true;
    crossedFinishLine = millis();
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

    changeMode(MEDIUM);
}

void LineFollower::updateButtons() {
    if (!gyroWasCalibrated) {
        return;
    }
    button1 = digitalRead(button1Pin);
    button2 = digitalRead(button2Pin);

    if (button1 && isButtonPressValid()) {
        toggleMotorsAreActive();
    }
    if (button2 && isButtonPressValid()) {
        switchMode();
        Serial.println("AAAAAAAAA");
    }
}

bool LineFollower::isButtonPressValid() {
    if (millis() > lastPressedButtonTime + 200) {
        lastPressedButtonTime = millis();
        return true;
    }
    return false;
}

void LineFollower::switchMode() {
    switch (currentMode) {
        case SLOW:
            currentMode = MEDIUM;
            break;
        case MEDIUM:
            currentMode = FAST;
            break;
        case FAST:
            currentMode = SLOW;
            break;
        default:
            break;
    }
}

void LineFollower::toggleMotorsAreActive() {
    shouldStop = false;
    delay(500);
    motorsAreActive = !motorsAreActive;
    shouldStop = false;
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

    if (numberOfActiveSensors == 0) {
        if (isOutOfLine == false) {
            isOutOfLine = true;
            outOfLineStartingTime = millis();
        }
    } else {
        isOutOfLine = false;
    }
    if (numberOfActiveSensors >= 4 && motorsAreActive) {
        lastCrossingTime = millis();
    }

    return lastValidSensorInput;
}

float LineFollower::calculateTargetRotSpeed(float error) {
    if (error == 0) return 0;
    const float absError = abs(error);
    const float signalError = error / absError;
    currentController = absError > 3
                            ? GYRO
                            : SENSOR;

    return (error * 70);
}

void LineFollower::updateMotors() {
    if (isOutOfLine) {
        pidResult = gyroPidResult * errorGain;

    } else {
        pidResult = sensorPidResult * 0.1;
    }
    const float turboedMotorOffset = getTurboOffset(motorOffset);
    leftMotorOutput = turboedMotorOffset - pidResult;
    rightMotorOutput = turboedMotorOffset + pidResult;

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

void LineFollower::printAll2() {
#ifdef SERIAL_DEBUG
    Serial.print("nOfRight: ");
    Serial.print(numberOfRightSignals);
    Serial.print("\t");
    Serial.print("Mode: ");
    Serial.print(currentMode);
    Serial.print("\t");
    Serial.println();

#endif
}

float LineFollower::calculateSensorReadingError(float error) {
    if (error == 0) return 0;
    float absError = abs(error);
    float errorSignal = error / absError;
    if (absError > 0 && absError <= 1) return error * 1.1;
    if (absError > 1 && absError <= 2) return error * 1.0;
    if (absError > 2 && absError <= 3) return error * 1.0;
    if (absError > 3) return error * 1.1;

    return error;
}

float LineFollower::calculateMotorOffset() {
    return maxMotorOffset;
    const float minMapRotSpeed = 5.0;
    const float maxMapRotSpeed = 90.0;
    const float constrainedRot = constrain(abs(rotSpeed), minMapRotSpeed, maxMapRotSpeed);

    return invertedMap(constrainedRot, minMapRotSpeed, maxMapRotSpeed, minMotorOffset, maxMotorOffset);
}

void LineFollower::triggeredInterruptRising(HelperSensorSide sensorSide) {
    if (!motorsAreActive || sensorSide == LEFT) return;

    const unsigned int timeNow = millis();
    if (timeNow - lastCrossingTime >= crossingTimeThreshold) {
        numberOfRightSignals++;
        if (numberOfRightSignals >= totalRightSignals) {
            endRun();
        }
    }
}

void LineFollower::triggeredInterruptFalling(HelperSensorSide sensorSide) {
    if (!motorsAreActive || sensorSide == LEFT) return;

    const unsigned int timeNow = micros();

    if (timeNow - lastInterrupt < 100) {
        lastInterrupt = timeNow;
        return;
    }
    lastInterrupt = timeNow;
    Serial.println("int");
    if (timeNow - lastCrossingTime >= crossingTimeThreshold) {
        numberOfRightSignals++;
        if (numberOfRightSignals >= totalRightSignals) {
            endRun();
        }
    }
}

void LineFollower::updateMode() {
    if (currentMode == SLOW) {
        minMotorOffset = 0.6;
        maxMotorOffset = 0.6;
        digitalWrite(led1Pin, LOW);
        digitalWrite(led2Pin, LOW);
    }
    if (currentMode == MEDIUM) {
        minMotorOffset = 0.4;
        maxMotorOffset = 0.8;
        digitalWrite(led1Pin, LOW);
        digitalWrite(led2Pin, HIGH);
    }
    if (currentMode == FAST) {
        minMotorOffset = 0.7;
        maxMotorOffset = 1.0;
        digitalWrite(led1Pin, HIGH);
        digitalWrite(led2Pin, HIGH);
    }
}

float LineFollower::getTurboOffset(float offset) {
    const float adjustedOffset = offset * speedMultiplier;
    return abs(sensorInput - sensorTarget) < 1 ? offset * 1.5 : offset;
}

void LineFollower::changeMode(Modes newMode) {
    currentMode = newMode;

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
    // digitalWrite(led2Pin, motorsAreActive ? HIGH : LOW);
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

    updateMode();

    rotSpeedTarget = calculateTargetRotSpeed(sensorTarget - sensorInput);
    rotSpeed = gyro->rotationSpeed;

    motorOffset = calculateMotorOffset();
    if (shouldStop) {
        if (millis() - crossedFinishLine >= 200) {
            motorsAreActive = false;
        }
    }
    /*
    if (isOutOfLine && millis() - outOfLineStartingTime >= 800) {
        motorsAreActive = false;
    }
    */

    if (motorsAreActive) {
        if (doOnceStart) {
            doOnceStart = false;
        }
        const bool processedRightHelper = sensorArray->rightSensProcessed;
        if (!lastRightHelper && processedRightHelper) {
            lastRightHelper = processedRightHelper;
            // triggeredInterrupt(RIGHT);
        }
        lastRightHelper = processedRightHelper;
        sensorPidResult = sensorPid->calculate(calculateSensorReadingError(sensorTarget - sensorInput));
        gyroPidResult = gyroPid->calculate(rotSpeedTarget - rotSpeed);
        updateMotors();
    } else {
        gyroPidResult = 0;
        sensorPidResult = 0;
        numberOfRightSignals = 0;

        if (shouldStop) {
            motors->brake();
        } else {
            motors->coast();
        }
    }
    /*
    const unsigned long runTime = millis() - crossedStartLine;
    if (runTime > 10000 && runTime < 13000) {
        speedMultiplier = 0.7;
    } else {
        speedMultiplier = 1.0;
    }
    */
    // printAll();
    // printAll2();
    delay(5);
}