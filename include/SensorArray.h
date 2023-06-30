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

#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include <Arduino.h>

#include "GlobalConsts.h"

class SensorArray {
   public:
    SensorArray(
        uint8_t multiplexerIOPin,
        uint8_t multiplexerS0Pin,
        uint8_t multiplexerS1Pin,
        uint8_t multiplexerS2Pin,
        uint8_t ledSelector1Pin,
        uint8_t ledSelector2Pin);

    void updateSensorsArray();

    void calibrateSensors();

    /*
        Returns the analog read of a sensor, receives an index;

        DOES NOT CHECK FOR THE LED
    */
    uint16_t analogReadSensorAt(uint8_t sensorIndex);

    uint16_t sensorsAnalog[N_OF_SENSORS];
    bool sensorsDigital[N_OF_SENSORS];

   private:
    // Turns the multiplexer pins to select a pin, receives an index
    void selectSensor(uint8_t sensorIndex);

    void updateDigitalValueOfSensors();

    uint8_t mplxIOPin;
    uint8_t mplxS0Pin;
    uint8_t mplxS1Pin;
    uint8_t mplxS2Pin;
    uint8_t ledSelec1Pin;
    uint8_t ledSelec2Pin;

    // Minimum analog read for each sensor
    uint16_t minRead[N_OF_SENSORS];

    // Maximum analog read for each sensor
    uint16_t maxRead[N_OF_SENSORS];

    // Threshold for each sensor
    uint16_t sensorsThreshold[N_OF_SENSORS];
};

#endif