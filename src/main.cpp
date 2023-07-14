#include <Arduino.h>

#include "GlobalConsts.h"
#include "Gyro.h"
#include "LineFollower.h"
#include "SensorArray.h"

#define MIO 9
#define MPLX_S0 13
#define MPLX_S1 14
#define MPLX_S2 21
#define LED_SELEC_1 10
#define LED_SELEC_2 12

#define SDA_PIN 16
#define SCL_PIN 15

#define STATUS_LED_1 41
#define STATUS_LED_2 42
#define INPUT_BTN_1 19
#define INPUT_BTN_2 20

SensorArray mySens(
    MIO,
    MPLX_S0,
    MPLX_S1,
    MPLX_S2,
    LED_SELEC_1,
    LED_SELEC_2);

Gyro myGyro;

LineFollower myLineFollower(
    mySens,
    myGyro,
    STATUS_LED_1,
    STATUS_LED_2,
    INPUT_BTN_1,
    INPUT_BTN_2);

void setup() {
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();

#ifdef SERIAL_DEBUG
    Serial.begin(115200);
    while (!Serial)
        delay(10);
#endif

    myLineFollower.initialize();
}

void loop() {
    myLineFollower.run();
}