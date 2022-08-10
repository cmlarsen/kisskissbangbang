#ifndef KKBB_Log_h
#define KKBB_Log_h
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "KKBB_Weapon.h"

struct displayStats {
  float volts;
  int actualRPM;
  int targetRPM;
  int inputPWM;
  int outputPWM;
  pidTuning pid;
  bool weaponTuningEnabled;
};

class KKBB_Log
{
public:
  KKBB_Log(uint8_t addr);
  void begin();
  void header(float voltage);
  void stats(displayStats s);

private:
  Adafruit_SSD1306 _display;
  bool _displayReady = false;
  uint8_t _addr;
};
#endif
