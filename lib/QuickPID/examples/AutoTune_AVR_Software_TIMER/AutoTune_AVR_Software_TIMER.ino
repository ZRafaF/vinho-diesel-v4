/******************************************************************************
   AutoTune AVR Software TIMER Example
   Circuit: https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter
 ******************************************************************************/
#include "Ticker.h" // https://github.com/sstaub/Ticker
#include "QuickPID.h"
void runPid();

const uint32_t sampleTimeUs = 10000; // 10ms
const byte inputPin = 0;
const byte outputPin = 3;
const int outputMax = 255;
const int outputMin = 0;

bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0

byte outputStep = 5;
byte hysteresis = 1;
int setpoint = 341;       // 1/3 of range for symetrical waveform
int output = 85;          // 1/3 of range for symetrical waveform

float Input, Output, Setpoint;
float Kp = 0, Ki = 0, Kd = 0;
bool pidLoop = false;
static boolean computeNow = false;

Ticker timer1(runPid, sampleTimeUs, 0, MICROS_MICROS);
QuickPID _myPID = QuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DOn, QuickPID::DIRECT);

void setup() {
  timer1.start();
  Serial.begin(115200);
  Serial.println();
  if (constrain(output, outputMin, outputMax - outputStep - 5) < output) {
    Serial.println(F("AutoTune test exceeds outMax limit. Check output, hysteresis and outputStep values"));
    while (1);
  }
  // Select one, reference: https://github.com/Dlloydev/QuickPID/wiki
  //_myPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PI);
  _myPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
  //_myPID.AutoTune(tuningMethod::TYREUS_LUYBEN_PI);
  //_myPID.AutoTune(tuningMethod::TYREUS_LUYBEN_PID);
  //_myPID.AutoTune(tuningMethod::CIANCONE_MARLIN_PI);
  //_myPID.AutoTune(tuningMethod::CIANCONE_MARLIN_PID);
  //_myPID.AutoTune(tuningMethod::AMIGOF_PID);
  //_myPID.AutoTune(tuningMethod::PESSEN_INTEGRAL_PID);
  //_myPID.AutoTune(tuningMethod::SOME_OVERSHOOT_PID);
  //_myPID.AutoTune(tuningMethod::NO_OVERSHOOT_PID);

  _myPID.autoTune->autoTuneConfig(outputStep, hysteresis, setpoint, output, QuickPID::DIRECT, printOrPlotter, sampleTimeUs);
}

void loop() {
  timer1.update();
  if (_myPID.autoTune) // Avoid dereferencing nullptr after _myPID.clearAutoTune()
  {
    switch (_myPID.autoTune->autoTuneLoop()) {
      case _myPID.autoTune->AUTOTUNE:
        Input = avg(_myPID.analogReadFast(inputPin));
        analogWrite(outputPin, Output);
        break;
      case _myPID.autoTune->TUNINGS:
        _myPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
        _myPID.SetMode(QuickPID::TIMER); // setup PID
        _myPID.SetSampleTimeUs(sampleTimeUs);
        _myPID.SetTunings(Kp, Ki, Kd, POn, DOn); // apply new tunings to PID
        Setpoint = 500;
        break;
      case _myPID.autoTune->CLR:
        if (!pidLoop) {
          _myPID.clearAutoTune(); // releases memory used by AutoTune object
          pidLoop = true;
        }
        break;
    }
  }
  if (pidLoop) {
    if (printOrPlotter == 0) { // plotter
      Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
      Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
      Serial.print("Output:");    Serial.print(Output);    Serial.println(",");
    }
    if (computeNow) {
      Input = _myPID.analogReadFast(inputPin);
      _myPID.Compute();
      analogWrite(outputPin, Output);
      computeNow = false;
    }
  }
}

void runPid() {
  computeNow = true;
}

float avg(int inputVal) {
  static int arrDat[16];
  static int pos;
  static long sum;
  pos++;
  if (pos >= 16) pos = 0;
  sum = sum - arrDat[pos] + inputVal;
  arrDat[pos] = inputVal;
  return (float)sum / 16.0;
}
