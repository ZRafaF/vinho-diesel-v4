#include <Arduino.h>

#include "GlobalConsts.h"
#include "SensorArray.h"

#define MIO 9
#define MPLX_S0 13
#define MPLX_S1 14
#define MPLX_S2 21
#define LED_SELEC_1 10
#define LED_SELEC_2 12
SensorArray mySens(MIO, MPLX_S0, MPLX_S1, MPLX_S2, LED_SELEC_1, LED_SELEC_2);

void setup() {
    Serial.begin(115200);

#ifdef SERIAL_DEBUG
    while (!Serial)
        delay(10);  // will pause Zero, Leonardo, etc until serial console opens

#endif
}

void loop() {
    Serial.println("update");
    mySens.updateSensorsArray();
    mySens.printAllAnalog();
    delay(200);
}