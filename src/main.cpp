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
#define LEFT_HELPER_SENS 1
#define RIGHT_HELPER_SENS 8

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

#define USE_ANALOG false
#define LINE_COLOR WHITE  // BLACK | WHITE

#if LINE_COLOR == WHITE
#define HELPER_INTERRUPT_MODE FALLING

#elif LINE_COLOR == BLACK
#define HELPER_INTERRUPT_MODE RISING

#else
#error "Invalid LINE_COLOR value"

#endif

SensorArray mySens(
    MIO,
    MPLX_S0,
    MPLX_S1,
    MPLX_S2,
    LED_SELEC_1,
    LED_SELEC_2,
    LEFT_HELPER_SENS,
    RIGHT_HELPER_SENS,
    SensorArray::LINE_COLOR,
    USE_ANALOG);

Gyro myGyro;

Tb6612fng myMotors(
    STBY,
    AIN_2,
    AIN_1,
    PWM_A,
    BIN_1,
    BIN_2,
    PWM_B);

PIDestal sensorsPid(2.0, 0.01, 3.0);
PIDestal gyroPid(0.90, 0.00001, 0.90);

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

void setSlowMode() {
    myLineFollower.changeMode(LineFollower::SLOW);
}
void setMediumMode() {
    myLineFollower.changeMode(LineFollower::MEDIUM);
}
void setFastMode() {
    myLineFollower.changeMode(LineFollower::FAST);
}

void leftSensInterrupt() {
    myLineFollower.triggeredInterrupt(LineFollower::LEFT);
}

void rightSensInterrupt() {
    myLineFollower.triggeredInterrupt(LineFollower::RIGHT);
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

    PIDestalRemoteBLE::FunctionPointer functions[] = {startStop, setSlowMode, setMediumMode, setFastMode};

    myRemotePid.setCallbackFunctions(functions, 4);
    // attachInterrupt(LEFT_HELPER_SENS, leftSensInterrupt, HELPER_INTERRUPT_MODE);
    // attachInterrupt(RIGHT_HELPER_SENS, rightSensInterrupt, HELPER_INTERRUPT_MODE);

#endif
}

void loop() {
    myLineFollower.run();
}