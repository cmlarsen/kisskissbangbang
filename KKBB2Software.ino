/**
 * @brief What's Next
 * - Display big warning when battery is low. (Flash screen or something)
 * - Display warning when transmitter is not connected.
 * - Build Servo
 *
 */

#include <movingAvg.h>
#include "KKBB_Voltage.h"
#include "KKBB_Receiver.h"
#include "KKBB_Weapon.h"
#include "KKBB_Log.h"

#define RX_PPM_PIN 0
#define DRIVE_ENCODER_A_PIN 1
#define DRIVE_ENCODER_B_PIN 2
#define VOLTAGE_PIN 3
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define WEAPON_TACH_PIN 6
#define DRIVE_ESC_A_PIN 7
#define DRIVE_ESC_B_PIN 8
#define WEAPON_ESC_PIN 9
#define DRIVE_HALL_PIN 10

#define PWM_MIN (1000) // Set the Minimum Speed in microseconds
#define PWM_MAX (2000)

#define RPM_MAX 15000

KKBB_Voltage voltage(VOLTAGE_PIN);
KKBB_Weapon weapon(WEAPON_TACH_PIN, WEAPON_ESC_PIN, RPM_MAX);
KKBB_Receiver rx(RX_PPM_PIN, PWM_MIN, PWM_MAX);
KKBB_Log display(0x3C);

movingAvg avgThrottleInput(20);

void handleWeaponTachInterrupt()
{
  weapon.calcRPMonInterrupt();
}

void setup()
{
  SerialUSB.begin(115200);
  display.begin();
  voltage.begin();
  weapon.begin();

  attachInterrupt(digitalPinToInterrupt(WEAPON_TACH_PIN), handleWeaponTachInterrupt, RISING);
  avgThrottleInput.begin();

  delay(500);
  weapon.enableTuningMode = true;
}

void loop()
{

  unsigned inputPWM = rx.readWeaponPWM();
  weapon.setInputPWM(inputPWM);
  weapon.update();



  display.stats({
    volts : voltage.readVoltage(),
    actualRPM : weapon.getRPM(),
    targetRPM : weapon.getTargetRPM(),
    inputPWM : inputPWM,
    outputPWM : weapon.getPWM(),
    pid : weapon.getPIDTuning(),
    weaponTuningEnabled: weapon.enableTuningMode
  });
}
