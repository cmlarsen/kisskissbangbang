
#include "Arduino.h"
#include "KKBB_Log.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

KKBB_Log::KKBB_Log(uint8_t addr) : _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1)
{
  _addr = addr;
}

void KKBB_Log::begin()
{
  // NOTE:  0x3C is the i2c address of the display;
  _displayReady = _display.begin(SSD1306_SWITCHCAPVCC, _addr);
  if (_displayReady)
  {
    _display.setTextSize(5);
    _display.setTextColor(WHITE);
    _display.setCursor(8, 16);
    _display.println("KKBB");
    SerialUSB.println("Kiss Kiss Bang Bang");
    _display.display();
    _display.setTextSize(1);
  }
  else
  {
    delay(1000);
    SerialUSB.println("No Display!");
  }
  return;
}

void KKBB_Log::header(float volts)
{
  SerialUSB.print("header: ");
  SerialUSB.println(_displayReady);

  if (!_displayReady)
    return;

  return;
}

void KKBB_Log::stats(displayStats s)
{
  _display.clearDisplay();
  _display.setCursor(0, 0);
  _display.print("KKBB");
  _display.setCursor(90, 0);
  _display.print(s.volts, 1);
  _display.println("v");
  if (s.weaponTuningEnabled)
  {
    _display.print("**Weapon Tuning**");
  }
  _display.setCursor(0, 16);

  _display.print("RPM ");
  _display.print("T:");
  _display.print(s.targetRPM);
  _display.setCursor(70, _display.getCursorY());
  _display.print("A:");
  _display.println(s.actualRPM);

  _display.println("");
  _display.print("PWM ");
  _display.print("I:");
  _display.print(s.inputPWM);
  _display.setCursor(70, _display.getCursorY());
  _display.print("O:");
  _display.println(s.outputPWM);

  _display.println("");
  _display.print("P");
  _display.print(s.pid.p, 3);
  _display.print(" I");
  _display.print(s.pid.i, 3);
  _display.print(" D");
  _display.println(s.pid.d, 3);
  _display.display();
}