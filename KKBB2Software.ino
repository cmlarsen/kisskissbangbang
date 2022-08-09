
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoRS485.h>
#include <ESC.h>
#include <PPMReader.h>
#include <movingAvg.h>
// #include <AutoPID.h>
#include <PID_v1.h>

/**
 * @brief Refactor plan
 * 1) Create Pin Defs section
 * 2) Extract domains to classes
 * 3) Abstract display and logging to optional class
 *
 */
#define PPM_PIN 0
#define DRIVE_ENCODER_A_PIN 1
#define DRIVE_ENCODER_A_PIN 2
#define UNUSED_PIN 3
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define WEAPON_HALL_PIN 6
#define UNUSED_UART_TX 7
#define VOLTAGE_PIN 8
#define WEP_ESC_PIN 9
#define DRIVE_HALL_PIN 10


movingAvg avgThrottleInput(20);
movingAvg avgRPM(30);
movingAvg avgVoltage(30);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// PPM channel layout


byte channelAmount = 8;
#define CHANNEL_THROTTLE 3
PPMReader ppm(PPM_PIN, channelAmount);


#define PWM_MIN (1000) // Set the Minimum Speed in microseconds
#define PWM_MAX (2000)
#define RPM_MIN 0
#define RPM_MAX 15000                                    // Set the Minimum Speed in microseconds
ESC weaponMotor(WEP_ESC_PIN, PWM_MIN, PWM_MAX, PWM_MIN); // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)
int escOutput = 0;

// Weapon Hall Sensor

#define PULSES_PER_REV 7
#define RPM_UPDATE_THRESHOLD 20000 // 1ms/10000 us
volatile float lastRPM = 0;
volatile uint32_t lastWeaponHallReading = 0;
volatile bool motordir = 1; // 0 reverse, 1 forward

double outputPWM;
double targetRPM;
double actualRPM;

// input/output variables passed by reference, so they are updated automatically
//  AutoPID myPID(&actualRPM, &targetRPM, &outputPWM, PWM_MIN, PWM_MAX, KP, KI, KD);
double Kp = 0.03;
double Ki = 0.06;
double Kd = 0;
PID myPID(&actualRPM, &outputPWM, &targetRPM, Kp, Ki, Kd, DIRECT);

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

bool newData = false;

void setup()
{
  SerialUSB.begin(9600);

  pinMode(VOLTAGE_PIN, INPUT);

  SerialUSB.print("Looking for OLED...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    SerialUSB.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  SerialUSB.println("Found it!");
  delay(1000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("Kiss Kiss Bang Bang");
  SerialUSB.println("Kiss Kiss Bang Bang");
  display.display();
  delay(1000);

  weaponMotor.arm(); // Send the Arm value so the ESC will be ready to take commands
  display.setCursor(0, 16);
  display.setTextSize(3);
  display.println("Armed");
  display.setTextSize(1);
  SerialUSB.println("Weapon Armed");

  pinMode(WEAPON_HALL_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(WEAPON_HALL_PIN), weaponHallInterrupt, RISING);

  display.display();
  avgThrottleInput.begin();
  avgRPM.begin();
  avgVoltage.begin();

  // if temperature is more than 100 rpm below or above setpoint, OUTPUT will be set to min or max respectively
  //  myPID.setBangBang(1000);
  // set PID update interval to 4000ms
  //  myPID.setTimeStep(20);
  outputPWM = PWM_MIN;
  targetRPM = 0;
  actualRPM = 0;
  myPID.SetOutputLimits(PWM_MIN, PWM_MAX);

  myPID.SetMode(AUTOMATIC);
  delay(2000);
}

void loop()
{
  display.clearDisplay();

  recvWithEndMarker();

  if (micros() - lastWeaponHallReading < RPM_UPDATE_THRESHOLD)
  {
    actualRPM = avgRPM.reading(lastRPM);
  }
  else
  {
    actualRPM = 0;
  }

  unsigned throttleRAW = ppm.latestValidChannelValue(CHANNEL_THROTTLE, 0);
  unsigned throttle = constrain(throttleRAW, PWM_MIN, PWM_MAX);
  targetRPM = avgThrottleInput.reading(map(throttle, PWM_MIN, PWM_MAX, RPM_MIN, RPM_MAX));

  // myPID.run();  //call every loop, updates automatically at certain time interval
  myPID.Compute();

  weaponMotor.speed(outputPWM);

  // setWeaponRPM(throttle, actualRPM)
  printHeader();
  printPPMValues(throttle);
  printPWMValue(outputPWM);
  printRPM((int)actualRPM, (int)targetRPM);
  printSerialData();
  display.display();
}

void printHeader()
{
  analogReadResolution(10);
  display.setCursor(0, 0);
  display.print("KKBB");
  display.setCursor(90, 0);
  int aV = avgVoltage.reading(analogRead(VOLTAGE_PIN));
  float voltage = aV * 0.01612903;

  display.print(voltage, 1);
  display.println("v");
  SerialUSB.print("Voltage: ");
  SerialUSB.println(voltage);
  display.setCursor(0, 16);
  analogReadResolution(12);
}

void weaponHallInterrupt()
{
  motordir = digitalRead(WEAPON_HALL_PIN);

  uint32_t currA = micros();
  if (lastWeaponHallReading < currA)
  {
    // did not wrap around
    float rev = currA - lastWeaponHallReading; // us
    rev = 1.0 / rev;                           // rev per us
    rev *= 1000000;                            // rev per sec
    rev *= 60;                                 // rev per min
    // rev /= GEARING;             // account for gear ratio
    // rev /= ENCODERMULT;         // account for multiple ticks per rotation
    rev /= PULSES_PER_REV; // account for multiple ticks per rotation
    lastRPM = rev;
  }
  lastWeaponHallReading = currA;
}

void printRPM(int lastRPM, int targetRPM)
{

  display.print("Actual RPM: ");
  display.println(lastRPM);
  display.print("Target RPM: ");
  display.println(targetRPM);

  SerialUSB.print("Actual RPM: ");
  SerialUSB.println(lastRPM);
  SerialUSB.print("Target RPM: ");
  SerialUSB.println(targetRPM);
}
void printPPMValues(int throttle)
{

  display.print("Throttle: ");
  display.println(throttle);
}

void printPWMValue(int val)
{

  display.print("Weapon PWM: ");
  display.println(val);
  SerialUSB.print("Weapon PWM: ");
  SerialUSB.println(val);
}

void printSerialData()
{

  display.print("P:");
  display.println(Kp, 3);
  display.print("I:");
  display.print(Ki, 3);
  display.print(" D:");
  display.println(Kd, 3);

  // SerialUSB.print("Serial: ");
  // SerialUSB.println(serialP);
  // if (newData == true) {
  //   newData = false;
  // }
}

void recvWithEndMarker()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (SerialUSB.available() > 0 && newData == false)
  {
    rc = SerialUSB.read();
    SerialUSB.println("SerialAvailable");
    SerialUSB.println(rc);

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
    myPID.SetTunings(Kp, Ki, Kd);

    // if (rc != endMarker) {
    //   receivedChars[ndx] = rc;
    //   ndx++;
    //   if (ndx >= numChars) {
    //     ndx = numChars - 1;
    //   }
    // } else {
    //   receivedChars[ndx] = '\0';  // terminate the string
    //   ndx = 0;
    //   newData = true;
    // }
  }
}