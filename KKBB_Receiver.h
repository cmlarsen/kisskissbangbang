#ifndef KKBB_Receiver_h
#define KKBB_Receiver_h
#include <PPMReader.h>

#include "Arduino.h"

class KKBB_Receiver
{
public:
  KKBB_Receiver(int pin, int pwm_min, int pwm_max);
  unsigned readWeaponPWM();

private:
  int _pwm_min;
  int _pwm_max;
  PPMReader _ppm;
};

#endif
