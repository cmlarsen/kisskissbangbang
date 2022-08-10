
#include "Arduino.h"
#include <movingAvg.h>
#include "KKBB_Voltage.h"

#define VOLT_PER_BIT 0.01612903

KKBB_Voltage::KKBB_Voltage(int pin) : _avg(30)
{
  pinMode(pin, INPUT);
  _pin = pin;
}

void KKBB_Voltage::begin()
{
  _avg.begin();
}

float KKBB_Voltage::readVoltage()
{

  int aV = _avg.reading(analogRead(_pin));
  float voltage = aV * VOLT_PER_BIT;
  return voltage;
}
