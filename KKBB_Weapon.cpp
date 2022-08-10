
#include "Arduino.h"
#include "KKBB_Weapon.h"
#include <movingAvg.h>
#include <ESC.h>
#include <PID_v1.h>

#define PULSES_PER_REV 7
#define RPM_UPDATE_THRESHOLD 20000
#define ARM_PWM_VALUE 1000
#define PWM_MIN 1000
#define PWM_MAX 2000

double _actualRPM= 0;
double _outputPWM = 1000;
double _targetRPM = 0;
PID _pid(&_actualRPM, &_outputPWM, &_targetRPM, 0.03, 0.06, 0.0, DIRECT);

KKBB_Weapon::KKBB_Weapon(int tachPin,
                         int escPin,
                         int rpmMax) : _avgInputPWM(10),
                                       _avgRPM(30),
                                       _esc(escPin, PWM_MIN, PWM_MAX, ARM_PWM_VALUE)
                                      //  ,_pid(&_actualRPM, &_outputPWM, &_targetRPM, _pidTuning.p, _pidTuning.i, _pidTuning.d, DIRECT)

{

  _tachPin = tachPin;
  _escPin = escPin;
  _rpmMax = rpmMax;

}

void KKBB_Weapon::begin()
{
  pinMode(_tachPin, INPUT_PULLUP);
  _outputPWM = PWM_MIN;
  _targetRPM = 0;
  _actualRPM = 0;
  _pid.SetOutputLimits(PWM_MIN, PWM_MAX);
  _pid.SetMode(AUTOMATIC);
  _esc.arm();
  _avgRPM.begin();
  _avgInputPWM.begin();
}

// Set this as the Interrupt ISR in the main setup()
void KKBB_Weapon::calcRPMonInterrupt()
{
  _motorDir = digitalRead(_tachPin);

  uint32_t currA = micros();
  if (_lastWeaponHallReading < currA)
  {
    // did not wrap around
    float rev = currA - _lastWeaponHallReading; // us
    rev = 1.0 / rev;                            // rev per us
    rev *= 1000000;                             // rev per sec
    rev *= 60;                                  // rev per min
    // rev /= GEARING;             // account for gear ratio
    // rev /= ENCODERMULT;         // account for multiple ticks per rotation
    rev /= PULSES_PER_REV; // account for multiple ticks per rotation
    _lastRPM = rev;
  }
  _lastWeaponHallReading = currA;
}

struct pidTuning KKBB_Weapon::getPIDTuning()
{
  pidTuning pt = {
    p : _pid.GetKp(),
    i : _pid.GetKi(),
    d : _pid.GetKd()
  };
  return pt;
}

int KKBB_Weapon::getPWM()
{
  return _outputPWM;
}

int KKBB_Weapon::getRPM()
{
  int actualRPM = 0;
  if (micros() - _lastWeaponHallReading < RPM_UPDATE_THRESHOLD)
  {
    actualRPM = _avgRPM.reading(_lastRPM);
  }
  return actualRPM;
}

int KKBB_Weapon::getTargetRPM(){
  return _targetRPM;
}

void KKBB_Weapon::setRPM(int targetRpm)
{
  _targetRPM = targetRpm;
}

void KKBB_Weapon::setInputPWM(int inputPWM)
{
  _targetRPM = _avgInputPWM.reading(map(inputPWM, PWM_MIN, PWM_MAX, 0, _rpmMax));
  // setRPM(_targetRPM);
}

void KKBB_Weapon::update()
{
  if(enableTuningMode){
    beginSerialTuner();
    SerialUSB.print(">Target RPM:");
    SerialUSB.println(_targetRPM);
    SerialUSB.print(">Actual RPM:");
    SerialUSB.println(_actualRPM);
  }

  _actualRPM = getRPM();
  _pid.Compute();
  _esc.speed(_outputPWM);
}

void KKBB_Weapon::beginSerialTuner()
{
  char rc;

  while (SerialUSB.available() > 0)
  {
    rc = SerialUSB.read();

    double Kp = _pid.GetKp();
    double Ki = _pid.GetKi();
    double Kd = _pid.GetKd();
    if (rc == 'p')
    {
      Kp = SerialUSB.parseFloat();
    }

    if (rc == 'i')
    {
      Ki = SerialUSB.parseFloat();
    }
    if (rc == 'd')
    {
      Kd = SerialUSB.parseFloat();
    }
    _pid.SetTunings(Kp, Ki, Kd);
  }
}