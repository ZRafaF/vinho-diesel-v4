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

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#define USE_BLUETOOTH

#include <Arduino.h>

#include "GlobalConsts.h"
#include "Gyro.h"
#include "PIDestal.h"
#include "PIDestalRemoteBLE.h"
#include "SensorArray.h"
#include "TB6612FNG.h"

float invertedMap(float input, float inMin, float inMax, float outMin, float outMax);

class LineFollower {
   public:
    enum ControllerType {
        SENSOR,
        GYRO
    };

    enum HelperSensorSide {
        LEFT,
        RIGHT
    };

    LineFollower(
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
        uint8_t inputButton2);

    // Sets up every component, should be called on the main setup function
    void initialize();

    void run();

    // Prints all parameters
    void printAll();

    void toggleMotorsAreActive();

    void triggeredInterrupt(HelperSensorSide sensorSide);

   private:
    /*
        Receives an array of booleans representing the current
        reading of each sensor and returns the average of them.
    */
    float calculateInput(bool sensorsDigital[N_OF_SENSORS]);
    float calculateTargetRotSpeed(float error);
    void updateMotors();

    void updateButtons();

    float calculateSensorReadingError(float error);

    float calculateMotorOffset();

    SensorArray* sensorArray;
    PIDestal* sensorPid;
    PIDestal* gyroPid;
#ifdef USE_BLUETOOTH
    PIDestalRemoteBLE* remotePid;
#endif
    Gyro* gyro;
    Tb6612fng* motors;

    float sensorTarget = 3.5f;          // Target
    float sensorInput;                  // Input
    float lastValidSensorInput = 3.5f;  // Last input

    float sensorPidResult = 0;
    float pidResult = 0;

    float gyroPidResult = 0;
    float errorGain = 0.01;

    float leftMotorOutput;
    float rightMotorOutput;
    bool motorsAreActive = false;
    unsigned long lastPressedButtonTime = 0;

    // Millis time of the last intersection
    unsigned long lastCrossingTime = 0;
    uint16_t crossingTimeThreshold = 1000;
    uint8_t numberOfRightSignals = 0;
    uint8_t totalRightSignals = 1;

    bool shouldStopOnTrigger = false;

    float motorOffset = 0.5;
    float motorClamp = 1;

    float rotSpeed;        // Speed of rotation
    float rotSpeedTarget;  // Speed of rotation
    float rotSpeedThreshold = 90.0f;

    uint8_t
        led1Pin,
        led2Pin,
        button1Pin,
        button2Pin;

    bool button1 = false;
    bool button2 = true;

    bool gyroWasCalibrated = false;

    bool isOutOfLine = true;

    bool crossedFinishLine = false;
    unsigned long beginningOfTheEndTime = 0;

    ControllerType currentController = SENSOR;
};

#endif  // LINE_FOLLOWER_H