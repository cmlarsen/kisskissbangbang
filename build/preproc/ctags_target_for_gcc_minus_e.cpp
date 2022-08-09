# 1 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino"

# 3 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 4 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 5 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 6 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 7 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 8 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
# 9 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2
// #include <AutoPID.h>
# 11 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino" 2

/**

 * @brief Refactor plan

 * 1) Create Pin Defs section

 * 2) Extract domains to classes

 * 3) Abstract display and logging to optional class

 *

 */
# 22 "/Users/caleb/Documents/Robots/Bots/KissKissBangBang Mk2/KKBB2Software/KKBB2Software.ino"
movingAvg avgThrottleInput(20);
movingAvg avgRPM(30);
movingAvg avgVoltage(30);




// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(128 /* OLED display width, in pixels*/, 64 /* OLED display height, in pixels*/, &Wire, -1);

// PPM channel layout

byte ppmInterruptPin = 0;
byte channelAmount = 8;
PPMReader ppm(ppmInterruptPin, channelAmount);






ESC weaponMotor(9 /* OLED display height, in pixels*/, (1000) /* Set the Minimum Speed in microseconds*/, (2000), (1000) /* Set the Minimum Speed in microseconds*/); // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)
int escOutput = 0;

// Weapon Hall Sensor



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
PID myPID(&actualRPM, &outputPWM, &targetRPM, Kp, Ki, Kd, 0);

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

bool newData = false;

void setup()
{
  SerialUSB.begin(9600);

  pinMode(8, (0x0));

  SerialUSB.print("Looking for OLED...");
  if (!display.begin(0x02 /*|< Gen. display voltage from 3.3V*/, 0x3C))
  { // Address 0x3C for 128x64
    SerialUSB.println((reinterpret_cast<const __FlashStringHelper *>(("SSD1306 allocation failed"))));
    for (;;)
      ;
  }
  SerialUSB.println("Found it!");
  delay(1000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(1 /*|< Draw 'on' pixels*/ /*|< Draw 'on' pixels*/);
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

  pinMode(6, (0x2));

  attachInterrupt(( 6 ), weaponHallInterrupt, 4);

  display.display();
  avgThrottleInput.begin();
  avgRPM.begin();
  avgVoltage.begin();

  // if temperature is more than 100 rpm below or above setpoint, OUTPUT will be set to min or max respectively
  //  myPID.setBangBang(1000);
  // set PID update interval to 4000ms
  //  myPID.setTimeStep(20);
  outputPWM = (1000) /* Set the Minimum Speed in microseconds*/;
  targetRPM = 0;
  actualRPM = 0;
  myPID.SetOutputLimits((1000) /* Set the Minimum Speed in microseconds*/, (2000));

  myPID.SetMode(1);
  delay(2000);
}

void loop()
{
  display.clearDisplay();

  recvWithEndMarker();

  if (micros() - lastWeaponHallReading < 20000 /* 1ms/10000 us*/)
  {
    actualRPM = avgRPM.reading(lastRPM);
  }
  else
  {
    actualRPM = 0;
  }

  unsigned throttleRAW = ppm.latestValidChannelValue(3, 0);
  unsigned throttle = ((throttleRAW)<((1000) /* Set the Minimum Speed in microseconds*/)?((1000) /* Set the Minimum Speed in microseconds*/):((throttleRAW)>((2000))?((2000)):(throttleRAW)));
  targetRPM = avgThrottleInput.reading(map(throttle, (1000) /* Set the Minimum Speed in microseconds*/, (2000), 0, 15000 /* Set the Minimum Speed in microseconds*/));

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
  int aV = avgVoltage.reading(analogRead(8));
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
  motordir = digitalRead(6);

  uint32_t currA = micros();
  if (lastWeaponHallReading < currA)
  {
    // did not wrap around
    float rev = currA - lastWeaponHallReading; // us
    rev = 1.0 / rev; // rev per us
    rev *= 1000000; // rev per sec
    rev *= 60; // rev per min
    // rev /= GEARING;             // account for gear ratio
    // rev /= ENCODERMULT;         // account for multiple ticks per rotation
    rev /= 7; // account for multiple ticks per rotation
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
