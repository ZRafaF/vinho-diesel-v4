/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "QuickPID.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop()
{
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  myQuickPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
