/**********************************************************************************
   QuickPID Library for Arduino - Version 2.4.6
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID Library and work on AutoTunePID class
   by gnalbandian (Gonzalo). Licensed under the MIT License.
 **********************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID.h"

/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, float POn = 1.0, float DOn = 0.0,
                   QuickPID::direction_t ControllerDirection = DIRECT) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = MANUAL;

  QuickPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QuickPID::SetControllerDirection(ControllerDirection);
  QuickPID::SetTunings(Kp, Ki, Kd, POn, DOn);

  lastTime = micros() - sampleTimeUs;
}

/* Constructor *********************************************************************
   To allow using Proportional on Error without explicitly saying so.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, direction_t ControllerDirection = DIRECT)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, pOn = 1.0, dOn = 0.0, ControllerDirection = DIRECT) {
}

/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QuickPID::Compute() {
  if (mode == MANUAL) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);
  if (mode == TIMER || timeChange >= sampleTimeUs) {
    float input = *myInput;
    float dInput = input - lastInput;
    error = *mySetpoint - input;
    if (controllerDirection == REVERSE) {
      error = -error;
      dInput = -dInput;
    }
    pmTerm = kpm * dInput;
    peTerm = kpe * error;
    iTerm = ki * error;
    dmTerm = kdm * dInput;
    deTerm = -kde * error;

    outputSum += iTerm;                                                           // include integral amount
    if (outputSum > outMax) outputSum -= outputSum - outMax;                      // early integral anti-windup at outMax
    else if (outputSum < outMin) outputSum += outMin - outputSum;                 // early integral anti-windup at outMin
    outputSum = constrain(outputSum, outMin, outMax);                             // integral anti-windup clamp
    outputSum = constrain(outputSum - pmTerm, outMin, outMax);                    // include pmTerm and clamp
    *myOutput = constrain(outputSum + peTerm + dmTerm - deTerm, outMin, outMax);  // totalize, clamp and drive the output

    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn = 1.0, float DOn = 0.0) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  pOn = POn;
  dOn = DOn;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  float SampleTimeSec = (float)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kpe = kp * pOn;
  kpm = kp * (1 - pOn);
  kde = kd * dOn;
  kdm = kd * (1 - dOn);
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered POn and DOn setting.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pOn, dOn);
}

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(..)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QuickPID::SetOutputLimits(int Min, int Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (mode != MANUAL) {
    *myOutput = constrain(*myOutput, outMin, outMax);
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

/* SetMode(.)*****************************************************************
  Sets the controller mode to MANUAL (0), AUTOMATIC (1) or TIMER (2)
  when the transition from MANUAL to AUTOMATIC or TIMER occurs, the
  controller is automatically initialized.
******************************************************************************/
void QuickPID::SetMode(mode_t Mode) {
  if (mode == MANUAL && Mode != MANUAL) { // just went from MANUAL to AUTOMATIC or TIMER
    QuickPID::Initialize();
  }
  mode = Mode;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  outputSum = constrain(outputSum, outMin, outMax);
}

/* SetControllerDirection(.)**************************************************
  The PID will either be connected to a DIRECT acting process (+Output leads
  to +Input) or a REVERSE acting process(+Output leads to -Input).
******************************************************************************/
void QuickPID::SetControllerDirection(direction_t ControllerDirection) {
  controllerDirection = ControllerDirection;
}

/* Status Functions************************************************************
  These functions query the internal state of the PID. They're here for display
  purposes. These are the functions the PID Front-end uses for example.
******************************************************************************/
float QuickPID::GetKp() {
  return dispKp;
}
float QuickPID::GetKi() {
  return dispKi;
}
float QuickPID::GetKd() {
  return dispKd;
}
float QuickPID::GetPterm() {
  return peTerm + pmTerm;
}
float QuickPID::GetIterm() {
  return iTerm;
}
float QuickPID::GetDterm() {
  return deTerm + dmTerm;
}
QuickPID::mode_t QuickPID::GetMode() {
  return  mode;
}
QuickPID::direction_t QuickPID::GetDirection() {
  return controllerDirection;
}

/* AutoTune Functions*********************************************************/

void QuickPID::AutoTune(tuningMethod tuningRule) {
  autoTune = new AutoTunePID(myInput, myOutput, tuningRule);
}

void QuickPID::clearAutoTune() {
  if (autoTune)
    delete autoTune;
}

AutoTunePID::AutoTunePID() {
  _input = nullptr;
  _output = nullptr;
  reset();
}

AutoTunePID::AutoTunePID(float *input, float *output, tuningMethod tuningRule) {
  AutoTunePID();
  _input = input;
  _output = output;
  _tuningRule = tuningRule;
}

void AutoTunePID::reset() {
  _tLast = 0;
  _t0 = 0;
  _t1 = 0;
  _t2 = 0;
  _t3 = 0;
  _Ku = 0.0;
  _Tu = 0.0;
  _td = 0.0;
  _kp = 0.0;
  _ki = 0.0;
  _kd = 0.0;
  _rdAvg = 0.0;
  _peakHigh = 0.0;
  _peakLow = 0.0;
  _autoTuneStage = 0;
}

void AutoTunePID::autoTuneConfig(const float outputStep, const float hysteresis, const float atSetpoint,
                                 const float atOutput, const bool dir, const bool printOrPlotter, uint32_t sampleTimeUs)
{
  _outputStep = outputStep;
  _hysteresis = hysteresis;
  _atSetpoint = atSetpoint;
  _atOutput = atOutput;
  _direction = dir;
  _printOrPlotter = printOrPlotter;
  _tLoop = constrain((sampleTimeUs / 8), 500, 16383);
  _autoTuneStage = STABILIZING;
}

byte AutoTunePID::autoTuneLoop() {
  if ((micros() - _tLast) <= _tLoop) return WAIT;
  else _tLast = micros();
  switch (_autoTuneStage) {
    case AUTOTUNE:
      return AUTOTUNE;
      break;
    case WAIT:
      return WAIT;
      break;
    case STABILIZING:
      if (_printOrPlotter == 1) Serial.print(F("Stabilizing →"));
      _t0 = millis();
      _peakHigh = _atSetpoint;
      _peakLow = _atSetpoint;
      (!_direction) ? *_output = 0 : *_output = _atOutput + (_outputStep * 2);
      _autoTuneStage = COARSE;
      return AUTOTUNE;
      break;
    case COARSE: // coarse adjust
      if (millis() - _t0 < 2000) {
        return AUTOTUNE;
        break;
      }
      if (*_input < (_atSetpoint - _hysteresis)) {
        (!_direction) ? *_output = _atOutput + (_outputStep * 2) : *_output = _atOutput - (_outputStep * 2);
        _autoTuneStage = FINE;
      }
      return AUTOTUNE;
      break;
    case FINE: // fine adjust
      if (*_input > _atSetpoint) {
        (!_direction) ? *_output = _atOutput - _outputStep : *_output = _atOutput + _outputStep;
        _autoTuneStage = TEST;
      }
      return AUTOTUNE;
      break;
    case TEST: // run AutoTune relay method
      if (*_input < _atSetpoint) {
        if (_printOrPlotter == 1) Serial.print(F(" AutoTune →"));
        (!_direction) ? *_output = _atOutput + _outputStep : *_output = _atOutput - _outputStep;
        _autoTuneStage = T0;
      }
      return AUTOTUNE;
      break;
    case T0: // get t0
      if (*_input > _atSetpoint) {
        _t0 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t0 →"));
        _inputLast = *_input;
        _autoTuneStage = T1;
      }
      return AUTOTUNE;
      break;
    case T1: // get t1
      if ((*_input > _atSetpoint) && (*_input > _inputLast)) {
        _t1 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t1 →"));
        _autoTuneStage = T2;
      }
      return AUTOTUNE;
      break;
    case T2: // get t2
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;
      if (_rdAvg > _atSetpoint + _hysteresis) {
        _t2 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t2 →"));
        (!_direction) ? *_output = _atOutput - _outputStep : *_output = _atOutput + _outputStep;
        _autoTuneStage = T3L;
      }
      return AUTOTUNE;
      break;
    case T3L: // t3 low cycle
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;
      if (_rdAvg < _atSetpoint - _hysteresis) {
        (!_direction) ? *_output = _atOutput + _outputStep : *_output = _atOutput - _outputStep;
        _autoTuneStage = T3H;
      }
      return AUTOTUNE;
      break;
    case T3H: // t3 high cycle, relay test done
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;
      if (_rdAvg > _atSetpoint + _hysteresis) {
        _t3 = micros();
        if (_printOrPlotter == 1) Serial.println(F(" t3 → done."));
        _autoTuneStage = CALC;
      }
      return AUTOTUNE;
      break;
    case CALC: // calculations
      _td = (float)(_t1 - _t0) / 1000000.0; // dead time (seconds)
      _Tu = (float)(_t3 - _t2) / 1000000.0; // ultimate period (seconds)
      _Ku = (float)(4 * _outputStep * 2) / (float)(3.14159 * sqrt (sq (_peakHigh - _peakLow) - sq (_hysteresis))); // ultimate gain
      if (_tuningRule == tuningMethod::AMIGOF_PID) {
        (_td < 0.1) ? _td = 0.1 : _td = _td;
        _kp = (0.2 + 0.45 * (_Tu / _td)) / _Ku;
        float Ti = (((0.4 * _td) + (0.8 * _Tu)) / (_td + (0.1 * _Tu)) * _td);
        float Td = (0.5 * _td * _Tu) / ((0.3 * _td) + _Tu);
        _ki = _kp / Ti;
        _kd = Td * _kp;
      } else { //other rules
        _kp = (float)(RulesContants[static_cast<uint8_t>(_tuningRule)][0] / 1000.0) * _Ku;
        _ki = (float)(RulesContants[static_cast<uint8_t>(_tuningRule)][1] / 1000.0) * (_Ku / _Tu);
        _kd = (float)(RulesContants[static_cast<uint8_t>(_tuningRule)][2] / 1000.0) * (_Ku * _Tu);
      }
      if (_printOrPlotter == 1) {
        // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
        if ((_Tu / _td + 0.0001) > 0.75) Serial.println(F("This process is easy to control."));
        else if ((_Tu / _td + 0.0001) > 0.25) Serial.println(F("This process has average controllability."));
        else Serial.println(F("This process is difficult to control."));
        Serial.print(F("Tu: ")); Serial.print(_Tu);    // Ultimate Period (sec)
        Serial.print(F("  td: ")); Serial.print(_td);  // Dead Time (sec)
        Serial.print(F("  Ku: ")); Serial.print(_Ku);  // Ultimate Gain
        Serial.print(F("  Kp: ")); Serial.print(_kp);
        Serial.print(F("  Ki: ")); Serial.print(_ki);
        Serial.print(F("  Kd: ")); Serial.println(_kd);
        Serial.println();
      }
      _autoTuneStage = TUNINGS;
      return AUTOTUNE;
      break;
    case TUNINGS:
      _autoTuneStage = CLR;
      return TUNINGS;
      break;
    case CLR:
      return CLR;
      break;
    default:
      return CLR;
      break;
  }
  return CLR;
}

void AutoTunePID::setAutoTuneConstants(float * kp, float * ki, float * kd) {
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
}

/* Utility************************************************************/
// https://github.com/avandalen/avdweb_AnalogReadFast
int QuickPID::analogReadFast(int ADCpin) {
#if defined(__AVR_ATmega328P__)
  byte ADCregOriginal = ADCSRA;
  ADCSRA = (ADCSRA & B11111000) | 5; // 32 prescaler
  int adc = analogRead(ADCpin);
  ADCSRA = ADCregOriginal;
  return adc;
#else
  return analogRead(ADCpin);
# endif
}
