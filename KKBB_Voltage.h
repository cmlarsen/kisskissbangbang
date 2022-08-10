#ifndef KKBB_Voltage_h
#define KKBB_Voltage_h
#include <movingAvg.h>

#include "Arduino.h"

class KKBB_Voltage
{
public:
  KKBB_Voltage(int pin);
  float readVoltage();
  void begin();

private:
  int _pin;
  movingAvg _avg;
};

#endif