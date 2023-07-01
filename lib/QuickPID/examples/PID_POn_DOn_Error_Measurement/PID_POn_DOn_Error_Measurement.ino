/**************************************************************************************
   Proportional and Derivative on the ratio of Error to Measurement Example
   Increasing the proportional on measurement setting will make the output move more
   smoothly when the setpoint is changed. Also, it can eliminate overshoot.
   Decreasing the derivative on measurement adds more derivative on error. This reduces
   reduce overshoot but may increase output spikes. Adjust to suit your requirements.
 **************************************************************************************/

#include "QuickPID.h"

//Define Variables we'll be connecting to
float Setpoint, Input, Output;
float POn = 1.0;   // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;   // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0

//Specify the links and initial tuning parameters
QuickPID myQuickPID(&Input, &Output, &Setpoint, 2, 5, 1, POn, DOn, QuickPID::DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(0);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop()
{
  Input = myQuickPID.analogReadFast(0);
  myQuickPID.Compute();
  analogWrite(3, Output);
}
