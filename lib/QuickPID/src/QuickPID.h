#pragma once
#ifndef QuickPID_h
#define QuickPID_h

enum class tuningMethod : uint8_t
{
  ZIEGLER_NICHOLS_PI,
  ZIEGLER_NICHOLS_PID,
  TYREUS_LUYBEN_PI,
  TYREUS_LUYBEN_PID,
  CIANCONE_MARLIN_PI,
  CIANCONE_MARLIN_PID,
  AMIGOF_PID,
  PESSEN_INTEGRAL_PID,
  SOME_OVERSHOOT_PID,
  NO_OVERSHOOT_PID
};

class AutoTunePID {
  public:
    AutoTunePID();
    AutoTunePID(float *input, float *output, tuningMethod tuningRule);
    ~AutoTunePID() {};

    void reset();
    void autoTuneConfig(const float outputStep, const float hysteresis, const float setpoint, const float output,
                        const bool dir = false, const bool printOrPlotter = false, uint32_t sampleTimeUs = 10000);
    byte autoTuneLoop();
    void setAutoTuneConstants(float* kp, float* ki, float* kd);
    enum atStage : byte { AUTOTUNE, WAIT, STABILIZING, COARSE, FINE, TEST, T0, T1, T2, T3L, T3H, CALC, TUNINGS, CLR };

  private:

    float *_input = nullptr;         // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *_output = nullptr;        // hard link between the variables and the PID, freeing the user from having
    // float *mySetpoint = nullptr;  // to constantly tell us what these values are. With pointers we'll just know.

    byte _autoTuneStage = 1;
    tuningMethod _tuningRule;
    bool _direction = false;
    bool _printOrPlotter = false;
    uint32_t _tLoop, _tLast, _t0, _t1, _t2, _t3;
    float _outputStep, _hysteresis, _atSetpoint, _atOutput;
    float _Ku, _Tu, _td, _kp, _ki, _kd, _rdAvg, _peakHigh, _peakLow, _inputLast;

    const uint16_t RulesContants[10][3] =
    { //ckp,  cki, ckd x 1000
      { 450,  540,   0 },  // ZIEGLER_NICHOLS_PI
      { 600,  176,  75 },  // ZIEGLER_NICHOLS_PID
      { 313,  142,   0 },  // TYREUS_LUYBEN_PI
      { 454,  206,  72 },  // TYREUS_LUYBEN_PID
      { 303, 1212,   0 },  // CIANCONE_MARLIN_PI
      { 303, 1333,  37 },  // CIANCONE_MARLIN_PID
      {   0,    0,   0 },  // AMIGOF_PID
      { 700, 1750, 105 },  // PESSEN_INTEGRAL_PID
      { 333,  667, 111 },  // SOME_OVERSHOOT_PID
      { 333,  100,  67 }   // NO_OVERSHOOT_PID
    };

}; // class AutoTunePID

class QuickPID {

  public:

    // controller mode
    typedef enum { MANUAL = 0, AUTOMATIC = 1, TIMER = 2 } mode_t;

    // DIRECT: intput increases when the error is positive. REVERSE: intput decreases when the error is positive.
    typedef enum { DIRECT = 0, REVERSE = 1 } direction_t;

    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float POn, float DOn, direction_t ControllerDirection);

    // Overload constructor with proportional ratio. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, direction_t ControllerDirection);

    // Sets PID mode to MANUAL (0), AUTOMATIC (1) or TIMER (2).
    void SetMode(mode_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Automatically determines and sets the tuning parameters Kp, Ki and Kd. Use this in the setup loop.
    void AutoTune(tuningMethod tuningRule);
    void clearAutoTune();

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(int Min, int Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

    // Overload for specifying proportional ratio.
    void SetTunings(float Kp, float Ki, float Kd, float POn, float DOn);

    // Sets the controller Direction or Action. DIRECT means the output will increase when the error is positive.
    // REVERSE means the output will decrease when the error is positive.
    void SetControllerDirection(direction_t ControllerDirection);

    // Sets the sample time in microseconds with which each PID calculation is performed. Default is 100000 µs.
    void SetSampleTimeUs(uint32_t NewSampleTimeUs);

    // PID Query functions ***********************************************************************************
    float GetKp();               // proportional gain
    float GetKi();               // integral gain
    float GetKd();               // derivative gain
    float GetPterm();            // proportional component of output
    float GetIterm();            // integral component of output
    float GetDterm();            // derivative component of output
    mode_t GetMode();            // MANUAL (0), AUTOMATIC (1) or TIMER (2)
    direction_t GetDirection();  // DIRECT (0) or REVERSE (1)

    int analogReadFast(int ADCpin);

    AutoTunePID *autoTune;

  private:

    void Initialize();

    float dispKp;       // tuning parameters for display purposes.
    float dispKi;
    float dispKd;
    float peTerm;
    float pmTerm;
    float iTerm;
    float deTerm;
    float dmTerm;

    float pOn;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
    float dOn;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
    float kp;           // (P)roportional Tuning Parameter
    float ki;           // (I)ntegral Tuning Parameter
    float kd;           // (D)erivative Tuning Parameter
    float kpe;          // proportional on error amount
    float kpm;          // proportional on measurement amount
    float kde;          // derivative on error amount
    float kdm;          // derivative on measurement amount

    float *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;    // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.

    mode_t mode = MANUAL;
    direction_t controllerDirection;
    uint32_t sampleTimeUs, lastTime;
    int outMin, outMax, error;
    int outputSum;
    float lastInput;
    bool inAuto;

}; // class QuickPID

#if (defined(ESP32) || defined(ARDUINO_ARCH_ESP32))
#include "analogWrite.h"
#endif
#endif // QuickPID.h
