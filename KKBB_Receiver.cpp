#include "Arduino.h"
#include "KKBB_Receiver.h"
#include <PPMReader.h>

#define CHANNEL_WEAPON 3

/**
 * TODO: Handle missing signal
 *
 */
const int channelCount = 8;

KKBB_Receiver::KKBB_Receiver(int pin, int pwm_min, int pwm_max) : _ppm(pin, channelCount)
{
  _pwm_min = pwm_min;
  _pwm_max = pwm_max;
}

unsigned KKBB_Receiver::readWeaponPWM()
{
  unsigned throttleRAW = _ppm.latestValidChannelValue(CHANNEL_WEAPON, 0);
  unsigned throttle = constrain(throttleRAW, _pwm_min, _pwm_max);
  return throttle;
}
