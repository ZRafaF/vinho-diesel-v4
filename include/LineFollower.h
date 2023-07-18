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

#include <Arduino.h>

#include "GlobalConsts.h"
#include "Gyro.h"
#include "PIDestal.h"
#include "PIDestalRemoteBLE.h"
#include "SensorArray.h"

class LineFollower {
   public:
    LineFollower(
        SensorArray& sensArrRef,
        Gyro& gyroRef,
        uint8_t statusLed1,
        uint8_t statusLed2,
        uint8_t inputButton1,
        uint8_t inputButton2);

    // Sets up every component, should be called on the main setup function
    void initialize();

    void run();

    // Prints all parameters
    void printAll();

   private:
    /*
        Receives an array of booleans representing the current
        reading of each sensor and returns the average of them.
    */
    float calculateInput(bool sensorsDigital[N_OF_SENSORS]);

    void updateButtons();

    SensorArray* sensorArray;
    PIDestal pid;
    Gyro* gyro;

    float setPoint = 3.5f;  // Target
    float input;            // PID input
    float output;           // PID output

    int16_t rotSpeed;        // Speed of rotation
    int16_t rotSpeedTarget;  // Speed of rotation

    float Kp = 2, Ki = 5, Kd = 1;

    uint8_t
        led1Pin,
        led2Pin,
        button1Pin,
        button2Pin;

    bool button1 = false;
    bool button2 = true;

    bool gyroWasCalibrated = false;
};

#endif  // LINE_FOLLOWER_H