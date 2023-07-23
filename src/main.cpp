#include <Arduino.h>

#include "GlobalConsts.h"
#include "Gyro.h"
#include "LineFollower.h"
#include "PIDestal.h"
#include "PIDestalRemoteBLE.h"
#include "SensorArray.h"
#include "TB6612FNG.h"

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

#define PWM_A 38
#define AIN_2 45
#define AIN_1 48
#define STBY 39
#define BIN_1 5
#define BIN_2 4
#define PWM_B 2

SensorArray mySens(
    MIO,
    MPLX_S0,
    MPLX_S1,
    MPLX_S2,
    LED_SELEC_1,
    LED_SELEC_2,
    SensorArray::BLACK,
    false);

Gyro myGyro;

Tb6612fng myMotors(
    STBY,
    AIN_1,
    AIN_2,
    PWM_A,
    BIN_1,
    BIN_2,
    PWM_B);

PIDestal sensorsPid(0.3, 0.0000001, 0.655);
PIDestal gyroPid(1.00, 0.00001, 0.01);

#ifdef USE_BLUETOOTH
PIDestal* pidArray[] = {&sensorsPid, &gyroPid};

PIDestalRemoteBLE myRemotePid(pidArray, 2);
#endif

LineFollower myLineFollower(
    mySens,
    myGyro,
    sensorsPid,
    gyroPid,
    myMotors,
#ifdef USE_BLUETOOTH
    myRemotePid,
#endif
    STATUS_LED_1,
    STATUS_LED_2,
    INPUT_BTN_1,
    INPUT_BTN_2);

void startStop() {
    myLineFollower.toggleMotorsAreActive();
}

void setup() {
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();

#ifdef SERIAL_DEBUG
    Serial.begin(115200);
    while (!Serial)
        delay(10);
#endif

    myLineFollower.initialize();

#ifdef USE_BLUETOOTH

    PIDestalRemoteBLE::FunctionPointer functions[] = {startStop};

    myRemotePid.setCallbackFunctions(functions, 1);

#endif
}

void loop() {
    myLineFollower.run();
}