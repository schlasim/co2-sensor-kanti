#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();

#include <ErriezMHZ19B.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
Ticker ledTicker;
Ticker printValuesTicker;
Ticker readSensorsTicker;
Ticker beepTicker;
Ticker stopToneTicker;
Ticker printDotTicker;

const char firmwareVersion[] = "v0.1.3";

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
void toggleErrorLed(void);

const int buzzerPin = D8;
const unsigned int beepFrequency = 1000;
const unsigned long beepDuration = 500;
const int numberOfMediumLowBeeps = 0;
const int numberOfMediumHighBeeps = 2;
const int numberOfHighBeeps = 3;

void friendlyBeep(void);
void friendlyBeep(int repetitions);
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
  co2MediumLow,
  co2MediumHigh,
  co2High
};
Co2Exposure co2Level = co2Low;
const int co2MediumLowThreshold = 800;
const int co2MediumHighThreshold = 1000;
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

void readSensors(void);
void readSensorsCallback(void);
bool readSensorsFlag = false;

void printValues(void);
void printValuesCallback(void);
bool printValuesFlag = false;

void setup()
{
  // enable leds
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

  // start blinking status led
  ledTicker.attach(1, toggleStatusLed);

  // test buzzer
  pinMode(buzzerPin, OUTPUT);
  analogWriteFreq(beepFrequency);
  analogWrite(buzzerPin, 512);
  delay(50);
  analogWrite(buzzerPin, LOW);

  // disable wifi
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // setup serial
  Serial.begin(9600);
  while (!Serial)
    ; // time to get serial running

  // setup display
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  updateFontParameters();
  y = ascent;
  u8g2.setCursor(0, y);
  u8g2.print(F("setup     "));
  u8g2.print(firmwareVersion);
  y = y + 1;
  u8g2.drawHLine(0, y, 64);
  u8g2.sendBuffer();
  newLine();

  u8g2.print(F("sht3x.."));
  u8g2.sendBuffer();
  if (!sht31.begin(0x44))
  {
    u8g2.print(F("ERROR!"));
    u8g2.sendBuffer();
    newLine();
    ledTicker.attach_ms(50, toggleErrorLed);
    while (1)
      delay(100);
  }
  u8g2.print(F(".ok"));
  u8g2.sendBuffer();
  newLine();

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
    ledTicker.attach_ms(50, toggleErrorLed);
    while (1)
      delay(100);
  }
  u8g2.print(F(".ok"));
  u8g2.sendBuffer();
  newLine();

  setupMhz19b();

  delay(2000);

  printValuesTicker.attach(2, printValuesCallback);
  readSensorsTicker.attach(5, readSensorsCallback);
  readSensorsFlag = true;
  printValuesFlag = true;
  readSensors();
  printValues();
}

void loop()
{
  readSensors();
  updateLeds();
  updateAlarm();
  printValues();
}

void toggleStatusLed(void)
{
  static bool state = true;
  digitalWrite(ledPin[yellow], state);
  state = !state;
}

void toggleErrorLed(void)
{
  static bool state = true;
  digitalWrite(ledPin[red], state);
  state = !state;
}

void printDot(void)
{
  u8g2.print(F("."));
  u8g2.sendBuffer();
}

void updateAlarm()
{
  static bool mediumLowAlarmHasFired = false;
  static bool mediumHighAlarmHasFired = false;
  static bool highAlarmHasFired = false;
  switch (co2Level)
  {
  case co2Low:
    mediumLowAlarmHasFired = false;
    mediumHighAlarmHasFired = false;
    highAlarmHasFired = false;
    break;
  case co2MediumLow:
    if (!mediumLowAlarmHasFired)
    {
      mediumLowAlarmHasFired = true;
      mediumHighAlarmHasFired = false;
      highAlarmHasFired = false;
      if (numberOfMediumLowBeeps > 0)
      {
        beepTicker.attach(1, friendlyBeep, (int)numberOfMediumLowBeeps);
      }
    }
    break;
  case co2MediumHigh:
    if (!mediumHighAlarmHasFired)
    {
      mediumHighAlarmHasFired = true;
      highAlarmHasFired = false;
      if (numberOfMediumHighBeeps > 0)
      {
        beepTicker.attach(1, friendlyBeep, (int)numberOfMediumHighBeeps);
      }
    }
    break;
  case co2High:
    if (!highAlarmHasFired)
    {
      highAlarmHasFired = true;
      if (numberOfHighBeeps > 0)
      {
        beepTicker.attach(1, friendlyBeep, (int)numberOfHighBeeps);
      }
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
  case co2MediumLow:
    setLed(green, ledOff);
    setLed(yellow, ledOn);
    setLed(red, ledOff);
    break;
  case co2MediumHigh:
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

void friendlyBeep(int repetitions)
{
  static int cnt = 0;
  if (repetitions - cnt > 0)
  {
    playTone(buzzerPin, beepFrequency, beepDuration);
    ++cnt;
  }
  else
  {
    beepTicker.detach();
    cnt = 0;
  }
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
  stopToneTicker.once_ms(duration, stopTone);
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
  if (readSensorsFlag)
  {
    mhzCo2 = mhz19b.readCO2();
    // mhzCo2 = mhzCo2 + 100; // for debugging
    if (mhzCo2 < co2MediumLowThreshold)
    {
      co2Level = co2Low;
    }
    else if ((co2MediumLowThreshold <= mhzCo2) && (mhzCo2 < co2MediumHighThreshold))
    {
      co2Level = co2MediumLow;
    }
    else if ((co2MediumHighThreshold <= mhzCo2) && (mhzCo2 < co2HighThreshold))
    {
      co2Level = co2MediumHigh;
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
    readSensorsFlag = false;
  }
}

void readSensorsCallback(void)
{
  readSensorsFlag = true;
}

void printValues()
{
  if (printValuesFlag)
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
    printValuesFlag = false;
  }
}

void printValuesCallback(void)
{
  printValuesFlag = true;
}

void setupMhz19b()
{
  char firmwareVersion[5];

  // Initialize serial port to print diagnostics and CO2 output
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
  printDotTicker.attach(15, printDot);
  setLed(yellow, ledOn);
  printDot();
  while (mhz19b.isWarmingUp())
  {
    delay(100);
  };
  setLed(yellow, ledOff);
  ledTicker.detach();
  printDotTicker.detach();
  u8g2.print(F(".ok"));
  u8g2.sendBuffer();

  delay(2000);

  u8g2.clearBuffer();
  y = ascent;
  u8g2.setCursor(0, y);
  u8g2.print(F("mh-z19b info"));
  y = y + 1;
  u8g2.drawHLine(0, y, 64);
  newLine();
  u8g2.sendBuffer();

  // Optional: Print firmware version
  Serial.print(F("  Firmware: "));
  mhz19b.getVersion(firmwareVersion, sizeof(firmwareVersion));
  Serial.println(firmwareVersion);
  u8g2.print(F("firmware: "));
  u8g2.print(firmwareVersion);
  newLine();
  u8g2.sendBuffer();
  // Optional: Set CO2 range 2000ppm or 5000ppm (default) once
  // Serial.print(F("Set range..."));
  mhz19b.setRange2000ppm();
  // mhz19b.setRange5000ppm();

  // Optional: Print operating range
  int range = mhz19b.getRange();
  Serial.print(F("  Range: "));
  Serial.print(range);
  Serial.println(F("ppm"));
  u8g2.print(F("range: "));
  u8g2.print(range);
  u8g2.print(F(" ppm"));
  newLine();
  u8g2.sendBuffer();

  // Optional: Set automatic calibration on (true) or off (false) once
  // Serial.print(F("Set auto calibrate..."));
  mhz19b.setAutoCalibration(true);

  // Optional: Print Automatic Baseline Calibration status
  int8_t autoCalibration = mhz19b.getAutoCalibration();
  Serial.print(F("  Auto calibrate: "));
  Serial.println(autoCalibration ? F("on") : F("off"));
  u8g2.print(F("auto cal: "));
  u8g2.print(autoCalibration ? F("on") : F("off"));
  newLine();
  u8g2.sendBuffer();

  delay(2000);
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
