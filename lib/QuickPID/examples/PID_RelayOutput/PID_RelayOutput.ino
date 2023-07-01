/*************************************************************
   PID Relay Output Example
   Same as basic example, except that this time, the output
   is going to a digital pin which (we presume) is controlling
   a relay.  The pid is designed to Output an analog value,
   but the relay can only be On/Off.

   To connect them together we use "time proportioning
   control", essentially a really slow version of PWM.
   First we decide on a window size (5000mS for example).
   We then set the pid to adjust its output between 0 and that
   window size. Lastly, we add some logic that translates the
   PID output into "Relay On Time" with the remainder of the
   window being "Relay Off Time". The minWindow setting is a
   floor (minimum time) the relay would be on.
 *************************************************************/

#include "QuickPID.h"

#define PIN_INPUT 0
#define RELAY_PIN 6

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;
float POn = 1.0;   // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;   // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DOn, QuickPID::DIRECT);

unsigned int WindowSize = 5000;
unsigned int minWindow = 500;
unsigned long windowStartTime;

void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myQuickPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime >= WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
    myQuickPID.Compute();
  }
  if (((unsigned int)Output > minWindow) && ((unsigned int)Output < (millis() - windowStartTime))) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);
}
