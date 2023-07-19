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

#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <Wire.h>

#include "GlobalConsts.h"
#include "MPU6050.h"

struct Vec3 {
    int16_t x, y, z;

    Vec3() {
        x = 0;
        y = 0;
        z = 0;
    };

    Vec3(int16_t _x, int16_t _y, int16_t _z) {
        x = _x;
        y = _y;
        z = _z;
    };
};

class Gyro {
   public:
    Gyro();

    // Initializes the I2C connection
    void initialize();

    /*
    Calibrates the gyroscope

    Return TRUE if it succeeded, else returns FALSE

    WARNING: The only parameter checked is the gyro.z
    */
    bool calibrate();

    // Prints the gyro and accel readings
    void printReadings();

    // Updates the Accelerometer and Gyroscope vector
    void update();

    // The only used parameter is the gyroscope z
    Vec3 accelerometer;
    Vec3 gyroscope;

    float rotationSpeed;

   private:
    MPU6050 accelGyro;
};
#endif