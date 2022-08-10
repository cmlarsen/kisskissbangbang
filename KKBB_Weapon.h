#ifndef KKBB_Weapon_h
#define KKBB_Weapon_h
#include <movingAvg.h>
#include <ESC.h>
#include <PID_v1.h>
#include "Arduino.h"

struct pidTuning
{
  double p, i, d;
};

class KKBB_Weapon
{
public:
  KKBB_Weapon(int tachPin, int escPin,  int rpmMax);
  void begin();
  void beginSerialTuner();
  void calcRPMonInterrupt();
  int getPWM();
  int getRPM();
  int getTargetRPM();
  struct pidTuning getPIDTuning();
  void setRPM(int targetRpm);
  void setInputPWM(int inputPWM);
  void update();
  bool enableTuningMode = false;

private:

  int _rpmMax;
  // double _targetRPM;
  // double _actualRPM;
  // double _outputPWM;
  int _tachPin;
  int _escPin;
  volatile boolean _motorDir = 1;
  volatile float _lastRPM = 0;
  volatile uint32_t _lastWeaponHallReading = 0;
  movingAvg _avgRPM;
  movingAvg _avgInputPWM;
  ESC _esc;
  // PID _pid;

  pidTuning _pidTuning = {0.03, 0.06, 0};
};
#endif