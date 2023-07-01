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

LineFollower::LineFollower(SensorArray& sensArrRef) : quickPID(&input, &output, &setPoint, Kp, Ki, Kd, QuickPID::DIRECT) {
    sensorArray = &sensArrRef;
    quickPID.SetMode(QuickPID::AUTOMATIC);
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
    sensorArray->updateSensorsArray();
    input = calculateInput(sensorArray->sensorsDigital);
    quickPID.Compute();

#ifdef SERIAL_DEBUG
    Serial.print("Input: ");
    Serial.print(input);
    Serial.print("Output: ");
    Serial.print(output);
    Serial.print("Target: ");
    Serial.print(setPoint);
#endif
}