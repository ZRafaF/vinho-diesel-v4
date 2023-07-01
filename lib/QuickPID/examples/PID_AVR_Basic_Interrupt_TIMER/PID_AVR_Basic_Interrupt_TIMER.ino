/********************************************************
   PID AVR Basic Interrupt TIMER Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include "TimerOne.h" // https://github.com/PaulStoffregen/TimerOne
#include "QuickPID.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3
const uint32_t sampleTimeUs = 100000; // 100ms
volatile bool computeNow = false;

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

void setup() {
  Timer1.initialize(sampleTimeUs); // initialize timer1, and set the time interval
  Timer1.attachInterrupt(runPid);  // attaches runPid() as a timer overflow interrupt

  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop() {
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
