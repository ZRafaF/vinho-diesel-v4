/********************************************************
   PID AVR Basic Software TIMER Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include "Ticker.h" // https://github.com/sstaub/Ticker
#include "QuickPID.h"
void runPid();

#define PIN_INPUT 0
#define PIN_OUTPUT 3
const uint32_t sampleTimeUs = 100000; // 100ms
static boolean computeNow = false;

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;

Ticker timer1(runPid, sampleTimeUs, 0, MICROS_MICROS);
QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

void setup() {
  timer1.start();

  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop() {
  timer1.update();
  if (computeNow) {
    Input = myQuickPID.analogReadFast(PIN_INPUT);
    myQuickPID.Compute();
    analogWrite(PIN_OUTPUT, Output);
    computeNow = false;
  }
}

void runPid() {
  computeNow = true;
}
