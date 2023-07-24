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
    enum LineColor {
        WHITE,
        BLACK
    };

    SensorArray(
        uint8_t multiplexerIOPin,
        uint8_t multiplexerS0Pin,
        uint8_t multiplexerS1Pin,
        uint8_t multiplexerS2Pin,
        uint8_t ledSelector1Pin,
        uint8_t ledSelector2Pin,
        uint8_t leftHelperPin,
        uint8_t rightHelperPin,
        LineColor colorOfTheLine,
        bool useAnalogSensors);

    void initialize();

    void updateSensorsArray();

    void calibrateSensors();

    void printAllRaw();
    void printAllProcessed();

    /*
        Returns the analog read of a sensor, receives an index;

        DOES NOT CHECK FOR THE LED
    */
    uint16_t readSensorAt(uint8_t sensorIndex);

    uint16_t sensorRaw[N_OF_SENSORS];
    bool sensorProcessed[N_OF_SENSORS];

    bool readsAnalog = true;
    LineColor lineColor = WHITE;

    bool leftSensRaw = false;
    bool rightSensRaw = false;

    bool leftSensProcessed = false;
    bool rightSensProcessed = false;

   private:
    // Turns the multiplexer pins to select a pin, receives an index
    void selectSensor(uint8_t sensorIndex);

    void processReadings();

    uint8_t _mplxIOPin;
    uint8_t _mplxS0Pin;
    uint8_t _mplxS1Pin;
    uint8_t _mplxS2Pin;
    uint8_t _ledSelec1Pin;
    uint8_t _ledSelec2Pin;
    uint8_t _leftHelperPin;
    uint8_t _rightHelperPin;

    // Minimum analog read for each sensor
    uint16_t minRead[N_OF_SENSORS];

    // Maximum analog read for each sensor
    uint16_t maxRead[N_OF_SENSORS];

    // Threshold for each sensor
    uint16_t sensorsThreshold[N_OF_SENSORS];
};

#endif