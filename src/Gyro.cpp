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
    Wire.begin();
    accelgyro.initialize();
}

void Gyro::update() {
    accelgyro.getMotion6(
        &accelerometer.x,
        &accelerometer.y,
        &accelerometer.z,
        &gyroscope.x,
        &gyroscope.y,
        &gyroscope.z);
}