//A4(SDA), A5(SCL)
//AM2320 1(VDD), 2(SDA), 3(GND), 4(SCL)

/*
    V0  BMP280 Temperature
    V1  DHT11 Humidity
    V2  MQ135 ppm value
    V3  MQ135 corrected ppm value
    V4  DHT11 Temperature
    V5  BMP280 Pressure in mmHg
    V6  GY-30 Light intensity value in lux
    V7  AM2320 Temperature
    V8  AM2320 Humidity
    V9  Motion Alarms
    V10 Sound Alarms
    V11 Widget LED for Red LED
    V12 Widget LED for Green LED
    V13 Widget LED Alarm
    V14 Widget LED Motion Alarm
    V15 Widget LED Sound Alarm
    V16 Widget LED Temperature Alarm
    V17 Widget LED Gas Alarm
    V18 Outdor Sensor Battery Sate
    V19 Outdor Sensor Signal Quality
    V20 Outdor Sensor Last Package Received Ago
*/

#include <BlynkSimpleStream.h>

#include "DHT.h"
#include <Adafruit_BMP280.h>
#include <MQ135.h>
#include <Wire.h>
//#include <BH1750.h>

#define pinRXINT        2
#define pinSound        3
#define pinDMQ135       4     //Gas Sensor
#define pinRCWL         5     //Micro Wave Sensor
#define DHTPIN          6
#define pinBuzzer       7
#define pinBuzzerOn     8
#define pinGuard        9
#define pinAlarm        10

#define pinLEDGreen     12
#define pinLEDRed       13

#define pinMQ135        A6
#define I2C_ADDR        9

#define DHTTYPE DHT11

int RCWLLastState = LOW;
int DMQ135LastState = LOW;

bool guardEnabled = false;
bool alarmEnabled = false;

int soundAlarms = 0;
int motionAlarms = 0;
int gasAlarms = 0;
int highTempAlarms = 0;
int lowTempAlarms = 0;
int soundAlarmsPushed = 0;
int motionAlarmsPushed = 0;
bool newPackageReceived = false;

volatile bool soundDetected = false;
bool motionDetected = false;
bool gasDetected = false;
bool highTempDetected = false;
bool lowTempDetected = false;

unsigned long timeLEDRed = 0;
unsigned long timeLEDGreen = 0;
unsigned long timeBuzzer = 0;
unsigned long timeAlarm = 0;
unsigned long packageReceivedAgo = 0;
volatile unsigned long timeEvent = 0;

char auth[] = "8a057e8ddce448cbbffe85ff05c64dde";

struct package
{
  uint8_t sender: 4, session: 2, type: 2;
  uint8_t received;
  uint16_t value;
};

typedef struct package Package;
Package data;

uint8_t buflen = sizeof(data);

volatile unsigned long timeRX = 0;

float tBMPSum = 0, tDHTSum = 0, humiditySum = 0, gasSum = 0, pressureSum = 0;
int tBMPCount = 0, tDHTCount = 0, humidityCount = 0, gasCount = 0, pressureCount = 0;

Adafruit_BMP280 bmp; //I2C
DHT dht(DHTPIN, DHTTYPE);
MQ135 gasSensor = MQ135(pinMQ135);
//BH1750 lightMeter;

bool BMP280Success;
//bool BH1750Success;


BlynkTimer pushTimer;
BlynkTimer syncTimer;


WidgetLED ledRed(V11);                //LED Red
WidgetLED ledGreen(V12);              //LED Green
WidgetLED ledAlarm(V13);              //LED Alarm
WidgetLED ledAlarmMotion(V14);        //LED Alarm Motion
WidgetLED ledAlarmSound(V15);         //LED Alarm Sound
WidgetLED ledAlarmTemperature(V16);   //LED Alarm Temperature
WidgetLED ledAlarmGas(V17);           //LED Alarm Gas

BLYNK_CONNECTED()
{
  ledRed.setValue(255 * digitalRead(pinLEDRed));
  ledGreen.setValue(255 * digitalRead(pinLEDGreen));
  ledAlarm.setValue(255 * digitalRead(pinAlarm));
  ledAlarmMotion.setValue(255 * (int)motionDetected);
  ledAlarmSound.setValue(255 * (int)soundDetected);
  ledAlarmTemperature.setValue(255 * (int)(highTempDetected || lowTempDetected));
  ledAlarmGas.setValue(255 * (int)gasDetected);
  Blynk.virtualWrite(V9, motionAlarms);
  Blynk.virtualWrite(V10, soundAlarms);
  motionAlarmsPushed = motionAlarms;
  soundAlarmsPushed = soundAlarms;
  Blynk.syncAll();
}


void onSoundDetect()
{
  soundDetected = true;
  timeEvent = millis();
}

void onPackageReceived()
{
  timeRX = millis();
}

void setup()
{
  pinMode(pinLEDRed, OUTPUT);
  pinMode(pinLEDGreen, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinDMQ135, INPUT);
  pinMode(pinBuzzerOn, INPUT);

  dht.begin();
  BMP280Success = bmp.begin(0x76);    //0x76 I2C Address
  Wire.begin();
  //  BH1750Success = lightMeter.begin();

  attachInterrupt(0, onPackageReceived, RISING);
  attachInterrupt(1, onSoundDetect, RISING);

  pushTimer.setInterval(1000L, pushData);
  syncTimer.setInterval(100L, syncState);

  Serial.begin(9600);
  Blynk.begin(Serial, auth);
}

void loop()
{
  unsigned long timeMillis = millis();

  if (timeLEDGreen > 0 && timeMillis - timeLEDGreen > 1000)
  {
    timeLEDGreen = 0;
    digitalWrite(pinLEDGreen, LOW);
  }

  int RCWLState = digitalRead(pinRCWL);
  if (RCWLState != RCWLLastState)
  {
    if (RCWLState == HIGH)
    {
      motionDetected = true;
      timeEvent = timeMillis;
    }
    RCWLLastState = RCWLState;
  }

  int DMQ135State = digitalRead(pinDMQ135);
  if (DMQ135State != DMQ135LastState)
  {
    if (DMQ135State == HIGH) gasDetected = true;
    else gasDetected = false;
    DMQ135LastState = DMQ135State;
  }

  if (timeRX > 0 && timeMillis - timeRX > 2500)
  {
    timeRX = 0;
    digitalWrite(pinLEDGreen, HIGH);
    timeLEDGreen = timeMillis;
    newPackageReceived = true;
  }

  int guardState = digitalRead(pinGuard);

  if (guardState == HIGH && !guardEnabled)
  {
    guardEnabled = true;
    ResetAlarms();
    timeLEDRed = 1;
    digitalWrite(pinLEDRed, HIGH);
  }
  else if (guardEnabled && guardState == LOW)
  {
    guardEnabled = false;
    DisableAlarm();
  }

  if (guardEnabled && !alarmEnabled)
  {
    if (soundDetected || motionDetected) alarmEnabled = true;
    if (soundDetected) soundAlarms++;
    if (motionDetected) motionAlarms++;
  }
  else if (!alarmEnabled)
  {
    if (gasDetected || highTempDetected || lowTempDetected) alarmEnabled = true;
    if (gasDetected) gasAlarms++;
    if (highTempDetected) highTempAlarms++;
    if (lowTempDetected) lowTempAlarms++;
  }

  if (alarmEnabled && timeAlarm == 0)
    EnableAlarm();
  else if (alarmEnabled && timeMillis - timeAlarm > 15000)
    DisableAlarm();
  else if (alarmEnabled)
  {
    BeepBuzzer(500);
    BlinkRed(500);
  }
  else if (!alarmEnabled && (soundAlarms > 0 || motionAlarms > 0 || gasAlarms > 0 || highTempAlarms > 0 || lowTempAlarms > 0))
  {
    BlinkRed(1000);
    if (timeMillis - timeEvent > 500)
    {
      motionDetected = false;
      soundDetected = false;
      timeEvent = 0;
    }
  }
  else if (!guardEnabled)
  {
    if ((motionDetected || soundDetected) && timeLEDRed == 0)
    {
      digitalWrite(pinLEDRed, HIGH);
      timeLEDRed = timeMillis;
    }
    if (timeLEDRed > 0 && timeMillis - timeLEDRed > 500)
    {
      digitalWrite(pinLEDRed, LOW);
      timeLEDRed = 0;
      motionDetected = false;
      soundDetected = false;
      timeEvent = 0;
    }
  }

  Blynk.run();
  pushTimer.run();
  syncTimer.run();
}

void ResetAlarms()
{
  DisableAlarm();
  soundAlarms = 0;
  motionAlarms = 0;
  gasAlarms = 0;
  highTempAlarms = 0;
  lowTempAlarms = 0;
}

void EnableAlarm()
{
  timeAlarm = millis();
  digitalWrite(pinAlarm, HIGH);
}

void DisableAlarm()
{
  timeAlarm = 0;
  alarmEnabled = false;
  soundDetected = false;
  motionDetected = false;
  digitalWrite(pinAlarm, LOW);
  digitalWrite(pinBuzzer, LOW);
  if (guardEnabled)
  {
    timeLEDRed = 1;
    digitalWrite(pinLEDRed, HIGH);
  }
  else
  {
    timeLEDRed = 0;
    digitalWrite(pinLEDRed, LOW);
  }
}


void syncState()
{
  int ledRedState = digitalRead(pinLEDRed);

  if (ledRedState == HIGH && ledRed.getValue() == 0)
    ledRed.on();
  else if (ledRedState == LOW && ledRed.getValue() == 255)
    ledRed.off();

  if (timeLEDGreen > 0 && ledGreen.getValue() == 0)
    ledGreen.on();
  else if (timeLEDGreen == 0 && ledGreen.getValue() == 255)
    ledGreen.off();

  if (alarmEnabled && ledAlarm.getValue() == 0)
    ledAlarm.on();
  if (!alarmEnabled && ledAlarm.getValue() == 255)
    ledAlarm.off();

  if (soundDetected && ledAlarmSound.getValue() == 0)
    ledAlarmSound.on();
  if (!soundDetected && ledAlarmSound.getValue() == 255)
    ledAlarmSound.off();

  if (motionDetected && ledAlarmMotion.getValue() == 0)
    ledAlarmMotion.on();
  if (!motionDetected && ledAlarmMotion.getValue() == 255)
    ledAlarmMotion.off();

  if (gasDetected && ledAlarmGas.getValue() == 0)
    ledAlarmGas.on();
  if (!gasDetected && ledAlarmGas.getValue() == 255)
    ledAlarmGas.off();

  if ((highTempDetected || lowTempDetected) && ledAlarmTemperature.getValue() == 0)
    ledAlarmTemperature.on();
  if (!highTempDetected && !lowTempDetected && ledAlarmTemperature.getValue() == 255)
    ledAlarmTemperature.off();
}

void processTH(float t, float h)
{
  if (isnan(t)) return;

  if (t < 15.0 && !lowTempDetected)
  {
    lowTempDetected = true;
    lowTempAlarms++;
  }
  else if (t > 26.0 && !highTempDetected)
  {
    highTempDetected = true;
    highTempAlarms++;
  }
  else if (lowTempDetected && t >= 15.0)
  {
    lowTempDetected = false;
  }
  else if (highTempDetected && t <= 26.0)
  {
    highTempDetected = false;
  }

  if (isnan(h)) return;

  gasSum += gasSensor.getCorrectedPPM(t, h);
  gasCount++;
  if (gasCount > 59)
  {
    Blynk.virtualWrite(V3, gasSum / gasCount);
    gasSum = 0;
    gasCount = 0;
  }
}

void pushData()
{
  float h = dht.readHumidity();
  if (!isnan(h))
  {
    humiditySum += h;
    humidityCount++;

    if (humidityCount > 59)
    {
      Blynk.virtualWrite(V1, humiditySum / humidityCount);
      humiditySum = 0;
      humidityCount = 0;
    }
  }

  float t = dht.readTemperature();
  if (!isnan(t))
  {
    tDHTSum += t;
    tDHTCount++;

    if (tDHTCount > 59)
    {
      Blynk.virtualWrite(V4, tDHTSum / tDHTCount);
      tDHTSum = 0;
      tDHTCount = 0;
    }
  }

  if (BMP280Success)
  {
    float tBMP = bmp.readTemperature();
    tBMPSum += tBMP;
    tBMPCount++;

    if (tBMPCount > 59)
    {
      Blynk.virtualWrite(V0, tBMPSum / tBMPCount);
      tBMPSum = 0;
      tBMPCount = 0;
    }

    pressureSum += bmp.readPressure() * 0.007501;
    pressureCount++;

    if (pressureCount > 59)
    {
      Blynk.virtualWrite(V5, pressureSum / pressureCount);
      pressureSum = 0;
      pressureCount = 0;
    }
    processTH(tBMP, h);
  }
  else
  {
    processTH(t, h);
  }

  Blynk.virtualWrite(V2, gasSensor.getPPM());

  //  if (BH1750Success)
  //    Blynk.virtualWrite(V6, lightMeter.readLightLevel());

  if (motionAlarmsPushed != motionAlarms)
  {
    Blynk.virtualWrite(V9, motionAlarms);
    motionAlarmsPushed = motionAlarms;
  }

  if (soundAlarmsPushed != soundAlarms)
  {
    Blynk.virtualWrite(V10, soundAlarms);
    soundAlarmsPushed = soundAlarms;
  }

  if (newPackageReceived)
  {
    newPackageReceived = false;
    packageReceivedAgo = 0;

    Wire.beginTransmission(I2C_ADDR);
    if (Wire.endTransmission() == 0)
    {
      Wire.requestFrom(I2C_ADDR, (int)buflen);      // Request N bytes from slave
      uint8_t bytes = Wire.available();
      if (bytes == buflen)
      {
        uint8_t buf[sizeof(data)];
        for (int i = 0; i < buflen; i++) buf[i] = Wire.read();

        data.sender = buf[0] & 0xf;
        data.session = (buf[0] >> 4) & 0x3;
        data.type = (buf[0] >> 6) & 0x3;
        data.received = buf[1];
        data.value = buf[2] | buf[3] << 8;

        if (data.sender == 1)
        {
          switch (data.type)
          {
            case 0:
              Blynk.virtualWrite(V18, data.value / 1000.0);
              break;
            case 1:
              if (data.value != 0xFFFF)
              {
                if (data.value & 0x8000) Blynk.virtualWrite(V7, -(int16_t)(data.value & 0x7fff) / 10.0);
                else Blynk.virtualWrite(V7, data.value / 10.0);
              }
              break;
            case 2:
              if (data.value != 0xFFFF)
              {
                Blynk.virtualWrite(V8, data.value / 10.0);
              }
              break;
          }
          Blynk.virtualWrite(V19, (float)data.received / 33.0 * 100.0);
        }
      }
    }
  }
  Blynk.virtualWrite(V20, packageReceivedAgo);
  packageReceivedAgo++;
}


void BlinkRed(int blinkDelay)
{
  unsigned long timeMillis = millis();
  int ledRedState = digitalRead(pinLEDRed);

  if (ledRedState == LOW && timeMillis - timeLEDRed > blinkDelay)
  {
    digitalWrite(pinLEDRed, HIGH);
    timeLEDRed = timeMillis;
  }
  else if (ledRedState == HIGH && timeMillis - timeLEDRed > blinkDelay)
  {
    digitalWrite(pinLEDRed, LOW);
    timeLEDRed = timeMillis;
  }
}

void BeepBuzzer(int beepDelay)
{
  int buzzerState = digitalRead(pinBuzzer);

  if (digitalRead(pinBuzzerOn) == LOW)
  {
    if (buzzerState = HIGH)
      digitalWrite(pinBuzzer, LOW);
    return;
  }

  unsigned long timeMillis = millis();

  if (buzzerState == LOW && timeMillis - timeBuzzer > beepDelay)
  {
    digitalWrite(pinBuzzer, HIGH);
    timeBuzzer = timeMillis;
  }
  else if (buzzerState == HIGH && timeMillis - timeBuzzer > beepDelay)
  {
    digitalWrite(pinBuzzer, LOW);
    timeBuzzer = timeMillis;
  }
}
