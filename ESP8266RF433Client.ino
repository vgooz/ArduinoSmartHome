#include <Wire.h>

//#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#define pinLED        D0
#define pinRXINT      D5
#define I2C_ADDR      9

/*
    V0 Sensor1 AM2320 Temperature
    V1 Sensor1 AM2320 Humidity
    V2 Sensor1 Battery Level
    V3 Sensor1 Signal Quality
    V4 Sensor1 Last Package Received Ago
*/

unsigned long timeLED = 0;

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

char auth[] = "807cd2f8cb1f41ad92398704330b4e3e";
char ssid[] = "GooZz2.4GHz";
char pass[] = "bluegene54321";

BlynkTimer pushTimer;
BlynkTimer syncTimer;
WidgetLED ledGreen(V12);              //LED Green
bool newPackage = false;
unsigned long lastPackageReceivedAgo = 0;

BLYNK_CONNECTED()
{
  ledGreen.setValue(255 * digitalRead(pinLED));
  Blynk.syncAll();
}

void onPackageReceived() {
  timeRX = millis();
}

void setup()
{
  pinMode(pinLED, OUTPUT);
  pinMode(pinRXINT, INPUT_PULLUP);
  
  Wire.begin();
  attachInterrupt(pinRXINT, onPackageReceived, RISING);

  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass, IPAddress(192, 168, 1, 100), 8181);

  pushTimer.setInterval(1000L, pushData);
  syncTimer.setInterval(100L, syncState);
}

void loop()
{
  unsigned long timeMillis = millis();

  if (timeLED > 0 && timeMillis - timeLED > 1000)
  {
    timeLED = 0;
    digitalWrite(pinLED, LOW);
  }

  if (timeRX > 0 && timeMillis - timeRX > 3000)
  {
    timeRX = 0;
    digitalWrite(pinLED, HIGH);
    timeLED = timeMillis;
    newPackage = true;
  }
  
  Blynk.run();
  pushTimer.run();
  syncTimer.run();
}

void syncState()
{
  if (timeLED > 0 && ledGreen.getValue() == 0) ledGreen.on();
  else if (timeLED == 0 && ledGreen.getValue() == 255) ledGreen.off();
}

void pushData()
{
  if (!newPackage)
  {
    lastPackageReceivedAgo++;
    Blynk.virtualWrite(V4, lastPackageReceivedAgo);
    return;
  }

  newPackage = false;
  lastPackageReceivedAgo = 0;
  Blynk.virtualWrite(V4, lastPackageReceivedAgo);

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
          case 0: //Battery Level
            Blynk.virtualWrite(V2, data.value / 1000.0);
            break;
          case 1: //AM2320 Temperature
            if (data.value != 0xFFFF)
            {
              if (data.value & 0x8000) Blynk.virtualWrite(V7, -(int16_t)(data.value & 0x7fff) / 10.0);
              else Blynk.virtualWrite(V0, data.value / 10.0);
            }
            break;
          case 2: //AM2320 Humidity
            if (data.value != 0xFFFF)
            {
              Blynk.virtualWrite(V1, data.value / 10.0);
            }
            break;
        }
        Blynk.virtualWrite(V3, (float)data.received / 33.0 * 100.0); //Signal Quality
      }
    }
  }
}
