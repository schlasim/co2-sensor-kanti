#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();

//#include <SHT31D_Universal.h>
//SHT31_Unified sht31;
#include <ErriezMHZ19B.h>
#include <SoftwareSerial.h> // Use software serial
#include <SimpleTimer.h>
SimpleTimer timer;
//timer.setInterval(1000, repeatMe);

enum LedName
{
  green,
  yellow,
  red,
  numberOfLeds
};
enum LedState
{
  ledOn,
  ledOff
};
const int ledPin[numberOfLeds]{D3, D4, D5};
void setLed(LedName name, LedState state);
void updateLeds(void);
void toggleStatusLed(void);

const int buzzerPin = D8;
const unsigned int beepFrequency = 2000;
const unsigned long beepDuration = 500;
// const int numberOfMedBeeps = 2;
const int numberOfHighBeeps = 3;

void friendlyBeep(void);
void playTone(int _pin, unsigned int frequency, unsigned long duration);
void stopTone(void);

U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0); // hardware
int maxCharHeight = 0;
int yInc = 0;
int ascent = 0;
int y = 0;
void newLine(void);
void updateFontParameters(void);

enum alarmState
{
  alarmOff,
  alarmOn
};
alarmState alarm = alarmOff;
void updateAlarm(void);

#define MHZ19B_TX_PIN D6
#define MHZ19B_RX_PIN D7
SoftwareSerial mhzSerial(MHZ19B_TX_PIN, MHZ19B_RX_PIN);
ErriezMHZ19B mhz19b(&mhzSerial);
void setupMhz19b(void);
void printDot(void);
void printErrorCode(int16_t result);
enum Co2Exposure
{
  co2Low,
  co2Medium,
  co2High
};
Co2Exposure co2Level = co2Low;
const int co2MediumThreshold = 800;
const int co2HighThreshold = 1200;



#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

float bmePress = 0.0;
float bmeAlt = 0.0;
float bmeTemp = 0.0;
float bmeHum = 0.0;

float shtTemp = 0.0;
float shtHum = 0.0;

int mhzCo2 = 0;

void readSensors();
void printValues();

void setup()
{
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ; // time to get serial running

  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function

  // Clear the buffer.
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  updateFontParameters();
  y = ascent;
  u8g2.setCursor(0, y);
  u8g2.print(F("setup"));
  u8g2.sendBuffer();
  newLine();

  pinMode(buzzerPin, OUTPUT);

  //  u8g2.print(F("leds.."));
  //  u8g2.sendBuffer();
  for (int i = 0; i < numberOfLeds; ++i)
  {
    pinMode(ledPin[i], OUTPUT);
    setLed(LedName(i), ledOn);
  }
  delay(2000);
  for (int i = 0; i < numberOfLeds; ++i)
  {
    setLed(LedName(i), ledOff);
  }
  //  u8g2.print(F(".ok"));
  //  u8g2.sendBuffer();
  //  newLine();

  u8g2.print(F("sht3x.."));
  u8g2.sendBuffer();
  if (!sht31.begin(0x44))
  {
    u8g2.print(F("ERROR!"));
    u8g2.sendBuffer();
    newLine();
    while (1)
      delay(10000);
  }
  u8g2.print(F(".ok"));
  u8g2.sendBuffer();
  newLine();

  //    u8g2.print(sht31.readTemperature());
  //    u8g2.print(F(" °C"));
  //    newLine();
  //    u8g2.print(sht31.readHumidity());
  //    u8g2.print(F(" %"));
  //    newLine();

  u8g2.print(F("bme280.."));
  u8g2.sendBuffer();
  delay(1000);

  unsigned status;

  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    u8g2.print(F("ERROR!"));
    u8g2.sendBuffer();
    newLine();
    while (1)
      delay(10000);
  }
  u8g2.print(F(".ok"));
  u8g2.sendBuffer();
  newLine();

  setupMhz19b();

  delay(2000);

  pinMode(D8, OUTPUT);

  timer.setInterval(2000L, printValues);
  timer.setInterval(5000L, readSensors);
  readSensors();
  printValues();
}

void loop()
{
  // put your main code here, to run repeatedly:
  timer.run();
  updateLeds();
  updateAlarm();
}

void toggleStatusLed(void)
{
  static bool state = true;
  digitalWrite(ledPin[yellow], state);
  state = !state;
}

void printDot(void)
{
  u8g2.print(F("."));
  u8g2.sendBuffer();
}

void updateAlarm()
{
  static bool mediumAlarmHasFired = false;
  static bool highAlarmHasFired = false;
  switch (co2Level)
  {
  case co2Low:
    mediumAlarmHasFired = false;
    highAlarmHasFired = false;
    break;
  case co2Medium:
    if (!mediumAlarmHasFired)
    {
      highAlarmHasFired = false;
      mediumAlarmHasFired = true;
      friendlyBeep();
    }
    break;
  case co2High:
    if (!highAlarmHasFired)
    {
      highAlarmHasFired = true;
      timer.setTimer(1000, friendlyBeep, numberOfHighBeeps);
    }
    break;
  default:
    setLed(green, ledOn);
    setLed(yellow, ledOn);
    setLed(red, ledOn);
    break;
  }
}

void updateLeds(void)
{
  switch (co2Level)
  {
  case co2Low:
    setLed(green, ledOn);
    setLed(yellow, ledOff);
    setLed(red, ledOff);
    break;
  case co2Medium:
    setLed(green, ledOff);
    setLed(yellow, ledOn);
    setLed(red, ledOff);
    break;
  case co2High:
    setLed(green, ledOff);
    setLed(yellow, ledOff);
    setLed(red, ledOn);
    break;
  default:
    setLed(red, ledOn);
    setLed(yellow, ledOff);
    setLed(red, ledOn);
    break;
  }
}

void setLed(LedName name, LedState state)
{
  digitalWrite(ledPin[name], state);
}

void friendlyBeep(void)
{
  playTone(buzzerPin, beepFrequency, beepDuration);
}

void playTone(int _pin, unsigned int frequency, unsigned long duration)
{
  pinMode(_pin, OUTPUT);
  analogWriteFreq(frequency);
  analogWrite(_pin, 512);
  timer.setTimeout(duration, stopTone);
}

void stopTone(void)
{
  analogWrite(buzzerPin, 0);
}

void newLine(void)
{
  y += yInc;
  u8g2.setCursor(0, y);
}

void updateFontParameters(void)
{
  maxCharHeight = u8g2.getMaxCharHeight();
  yInc = maxCharHeight + 1;
  ascent = u8g2.getAscent();
}

void readSensors()
{
  mhzCo2 = mhz19b.readCO2();
  if (mhzCo2 < co2MediumThreshold)
  {
    co2Level = co2Low;
  }
  else if ((co2MediumThreshold <= mhzCo2) && (mhzCo2 < co2HighThreshold))
  {
    co2Level = co2Medium;
  }
  else
  {
    co2Level = co2High;
  }

  bmePress = bme.readPressure() / 100.0F;
  bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  bmeTemp = bme.readTemperature();
  bmeHum = bme.readHumidity();

  shtTemp = sht31.readTemperature();
  shtHum = sht31.readHumidity();
}

void printValues()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB14_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  updateFontParameters();
  y = ascent;
  u8g2.setCursor(0, y);
  if (mhzCo2 < 1000)
  {
    u8g2.print(F(" "));
  }
  u8g2.print(mhzCo2);
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  updateFontParameters();
  u8g2.print(" ppm");
  y = 25;
  u8g2.setCursor(0, y);

  u8g2.print(bmeTemp, 1);
  u8g2.print(F(" °C  "));
  u8g2.print(shtTemp, 1);
  u8g2.print(F(" °C"));
  newLine();

  u8g2.print(bmeHum, 1);
  u8g2.print(F(" %   "));
  u8g2.print(shtHum, 1);
  u8g2.print(F(" %"));
  newLine();

  u8g2.print(bmePress, 1);
  u8g2.print(F(" hPa"));
  newLine();

  u8g2.print(bmeAlt, 1);
  u8g2.print(F(" m"));
  newLine();
  u8g2.sendBuffer();
}

//void readCo2() {
//  int co2Value;
//
//  /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even
//    if below background CO2 levels or above range (useful to validate sensor). You can use the
//    usual documented command with getCO2(false) */
//
//  co2Value = myMHZ19.getCO2(false);  // Request CO2 (as ppm)
//
//  Serial.print(F("CO2 (ppm): "));
//  Serial.println(co2Value);
//  Blynk.virtualWrite(V0, co2Value);
//
//  int8_t temperatureValue;
//  temperatureValue = myMHZ19.getTemperature();  // Request Temperature (as Celsius)
//  Serial.print(F("Temperature (C): "));
//  Serial.println(temperatureValue);
//  Blynk.virtualWrite(V1, temperatureValue);
//}

void setupMhz19b()
{
  char firmwareVersion[5];

  // Initialize serial port to print diagnostics and CO2 output
  Serial.begin(115200);
  Serial.println(F("\nErriez MH-Z19B CO2 Sensor example"));

  // Initialize senor software serial at fixed 9600 baudrate
  mhzSerial.begin(9600);

  // Optional: Detect MH-Z19B sensor (check wiring / power)
  u8g2.print(F("mh-z19b"));
  newLine();
  u8g2.sendBuffer();
  while (!mhz19b.detect())
  {
    Serial.println(F("Detecting MH-Z19B sensor..."));
    delay(2000);
  };

  // Sensor requires 3 minutes warming-up after power-on
  u8g2.print(F("warming up.."));
  newLine();
  u8g2.sendBuffer();
  timer.setInterval(1000, toggleStatusLed);
  timer.setInterval(15000, printDot);
  setLed(yellow, ledOn);
  printDot();
  while (mhz19b.isWarmingUp())
  {
    timer.run();
    delay(1);
  };
  setLed(yellow, ledOff);
  timer.deleteTimer(0);
  timer.deleteTimer(1);

  // Optional: Print firmware version
  Serial.print(F("  Firmware: "));
  mhz19b.getVersion(firmwareVersion, sizeof(firmwareVersion));
  Serial.println(firmwareVersion);

  // Optional: Set CO2 range 2000ppm or 5000ppm (default) once
  // Serial.print(F("Set range..."));
  // mhz19b.setRange2000ppm();
  // mhz19b.setRange5000ppm();

  // Optional: Print operating range
  Serial.print(F("  Range: "));
  Serial.print(mhz19b.getRange());
  Serial.println(F("ppm"));

  // Optional: Set automatic calibration on (true) or off (false) once
  // Serial.print(F("Set auto calibrate..."));
  // mhz19b.setAutoCalibration(true);

  // Optional: Print Automatic Baseline Calibration status
  Serial.print(F("  Auto calibrate: "));
  Serial.println(mhz19b.getAutoCalibration() ? F("On") : F("Off"));
  // OLD CODE:
  //  Serial.println(F("@MH: opening serial"));
  //  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start
  //  //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // (ESP32 Example) device to MH-Z19 serial start
  //  Serial.println(F("@MH: starting sensor"));
  //  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin().
  //  Serial.println(F("@MH: turning off auto calibration"));
  //  myMHZ19.autoCalibration(false);                              // Turn auto calibration ON (OFF autoCalibration(false))

  u8g2.print(F(".ok"));
  u8g2.sendBuffer();
  newLine();
}

void printErrorCode(int16_t result)
{
  // Print error code
  switch (result)
  {
  case MHZ19B_RESULT_ERR_CRC:
    Serial.println(F("CRC error"));
    break;
  case MHZ19B_RESULT_ERR_TIMEOUT:
    Serial.println(F("RX timeout"));
    break;
  default:
    Serial.print(F("Error: "));
    Serial.println(result);
    break;
  }
}
