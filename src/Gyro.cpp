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

#include "Gyro.h"

Gyro::Gyro() {
}
void Gyro::initialize() {
    accelGyro.initialize();

#ifdef SERIAL_DEBUG
    Serial.println(accelGyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif
}

bool Gyro::calibrate() {
    accelGyro.CalibrateAccel(6);
    accelGyro.CalibrateGyro(6);
#ifdef SERIAL_DEBUG
    accelGyro.PrintActiveOffsets();
#endif
    update();

    return abs(gyroscope.z) <= 500 ? true : false;
}

void Gyro::printReadings() {
#ifdef SERIAL_DEBUG
    Serial.print("a/g:\t");
    Serial.print(accelerometer.x);
    Serial.print("\t");
    Serial.print(accelerometer.y);
    Serial.print("\t");
    Serial.print(accelerometer.z);
    Serial.print("\t");
    Serial.print(gyroscope.x);
    Serial.print("\t");
    Serial.print(gyroscope.y);
    Serial.print("\t");
    Serial.println(gyroscope.z);
#endif
}

void Gyro::update() {
    accelGyro.getMotion6(
        &accelerometer.x,
        &accelerometer.y,
        &accelerometer.z,
        &gyroscope.x,
        &gyroscope.y,
        &gyroscope.z);
}