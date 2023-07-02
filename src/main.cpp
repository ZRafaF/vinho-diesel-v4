#include <Arduino.h>

#include "GlobalConsts.h"
#include "SensorArray.h"

void setup() {
    Serial.begin(115200);

#ifdef SERIAL_DEBUG
    while (!Serial)
        delay(10);  // will pause Zero, Leonardo, etc until serial console opens

#endif
}

void loop() {
    Serial.print("123123");
}