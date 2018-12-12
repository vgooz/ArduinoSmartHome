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
*/

#include <BlynkSimpleStream.h>

#include <VirtualWire.h>
#include "DHT.h"
//#include "Adafruit_AM2320.h"
#include <Adafruit_BMP280.h>
#include <MQ135.h>
#include <BH1750.h>

#define pinLEDRed       13
#define pinLEDGreen     12
#define pinRX           4
#define pinMQ135        A7
#define pinDMQ135       2     //Gas Sensor
#define pinBuzzer       9
#define pinSound        3
#define pinRCWL         5     //Micro Wave Sensor
#define DHTPIN          6
#define pinGuard        7
#define pinAlarm        8
#define pinResetAlarms  10

#define DHTTYPE DHT11

int RCWLLastState = LOW;

bool guardEnabled = false;
bool alarmsEnabled = false;
int soundAlarms = 0;
int motionAlarms = 0;
int gasAlarms = 0;
int tempHighAlarms = 0;
int tempLowAlarms = 0;

char auth[] = "8a057e8ddce448cbbffe85ff05c64dde";
char DEVID[37];

volatile unsigned long timeLEDRed = 0;
unsigned long timeLEDGreen = 0;
unsigned long timeBuzzer = 0;
unsigned long timeAlarm = 0;

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

Adafruit_BMP280 bmp; //I2C
//Adafruit_AM2320 am2320 = Adafruit_AM2320();
DHT dht(DHTPIN, DHTTYPE);
MQ135 gasSensor = MQ135(pinMQ135);
BH1750 lightMeter;

bool BMP280Success;
bool AM2320Success;
bool BH1750Success;


BlynkTimer timer;

WidgetLED ledRed(V11);                //LED Red
WidgetLED ledGreen(V12);              //LED Green
WidgetLED ledAlarm(V13);              //LED Alarm
WidgetLED ledAlarmMotion(V14);        //LED Alarm Motion
WidgetLED ledAlarmSound(V15);         //LED Alarm Sound
WidgetLED ledAlarmTemperature(V16);   //LED Alarm Temperature
WidgetLED ledAlarmGas(V17);           //LED Alarm Gas

void setup()
{
  pinMode(pinLEDRed, OUTPUT);
  pinMode(pinLEDGreen, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinSound, INPUT);
  pinMode(pinRCWL, INPUT);
  pinMode(pinGuard, INPUT);
  pinMode(pinAlarm, INPUT);
  pinMode(pinResetAlarms, INPUT);

  vw_setup(400);
  vw_set_rx_pin(pinRX);
  vw_rx_start();
  //  AM2320Success = am2320.begin();
  dht.begin();
  BMP280Success = bmp.begin(0x76); //0x76 I2C Address
  BH1750Success = lightMeter.begin();

  attachInterrupt(1, SoundDetected, RISING);

  Serial.begin(9600);
  Blynk.begin(Serial, auth);

  timer.setInterval(1000L, myTimerEvent);
}

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

void ResetAlarms()
{
  DisableAlarm();
  soundAlarms = 0;
  motionAlarms = 0;
  gasAlarms = 0;
  tempHighAlarms = 0;
  tempLowAlarms = 0;
  Blynk.virtualWrite(V9, motionAlarms);
  Blynk.virtualWrite(V10, soundAlarms);
}

void EnableAlarm()
{
  timeAlarm = millis();
  digitalWrite(pinAlarm, HIGH);
  ledAlarm.on();
}

void DisableAlarm()
{
  timeAlarm = 0;
  digitalWrite(pinAlarm, LOW);
  digitalWrite(pinBuzzer, LOW);
  ledAlarm.off();
  ledAlarmMotion.off();
  ledAlarmSound.off();
  ledAlarmTemperature.off();
  ledAlarmGas.off();
  if (guardEnabled)
  {
    digitalWrite(pinLEDRed, HIGH);
    ledRed.on();
  }  
  else
  {
    timeLEDRed = 0;
    digitalWrite(pinLEDRed, LOW);
    ledRed.off();
  }
}

void SoundDetected()
{
  timeLEDRed = 2;
}

void myTimerEvent()
{
  float t = -100, h = -1;

  if (BMP280Success)
  {
    t = bmp.readTemperature();

    Blynk.virtualWrite(V0, t);

    Blynk.virtualWrite(V5, bmp.readPressure() * 0.007501);

    // Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
  }

  Blynk.virtualWrite(V4, dht.readTemperature());

  h = dht.readHumidity();
  Blynk.virtualWrite(V1, h);

  /*
    if (AM2320Success)
    {
      t = am2320.readTemperature();
      Blynk.virtualWrite(V0, t);

      h = am2320.readHumidity();
      Blynk.virtualWrite(V1, h);
    }
  */

  Blynk.virtualWrite(V2, gasSensor.getPPM());

  if (t > -100 && h > 0)
  {
    float ppm = gasSensor.getCorrectedPPM(t, h);
    Blynk.virtualWrite(V3, ppm);
    if (t > 26.0)
    {
      tempHighAlarms++;
      EnableAlarm();
      ledAlarmTemperature.on();
    }
    else if (t < 15.0)
    {
      tempLowAlarms++;
      EnableAlarm();
      ledAlarmTemperature.on();
    }
    if (ppm > 1000.0)
    {
      gasAlarms++;
      EnableAlarm();
      ledAlarmGas.on();
    }
  }

  if (BH1750Success)
    Blynk.virtualWrite(V6, lightMeter.readLightLevel());
}


void BlinkRed(int blinkDelay)
{
  unsigned long timeMillis = millis();
  int ledRedState = digitalRead(pinLEDRed);

  if (ledRedState == LOW && timeMillis - timeLEDRed > blinkDelay)
  {
    digitalWrite(pinLEDRed, HIGH);
    timeLEDRed = timeMillis;
    ledRed.on();
  }
  else if (ledRedState == HIGH && timeMillis - timeLEDRed > blinkDelay)
  {
    digitalWrite(pinLEDRed, LOW);
    timeLEDRed = timeMillis;
    ledRed.off();
  }
}

void BeepBuzzer(int beepDelay)
{
  unsigned long timeMillis = millis();
  int buzzerState = digitalRead(pinBuzzer);

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

void loop()
{

  unsigned long timeMillis = millis();

  if (digitalRead(pinResetAlarms) == HIGH) ResetAlarms();

  int guardState = digitalRead(pinGuard);

  if (guardState == HIGH && !guardEnabled)
  {
    guardEnabled = true;
    ResetAlarms();
  }
  else if (guardEnabled && guardState == LOW)
  {
    guardEnabled = false;
    DisableAlarm();
  }

  if (guardEnabled && timeAlarm == 0)
  {
    if (timeLEDRed == 2)
    {
      soundAlarms++;
      EnableAlarm();
      ledAlarmSound.on();
      Blynk.virtualWrite(V10, soundAlarms);
    }
    int RCWLState = digitalRead(pinRCWL);
    if (RCWLState != RCWLLastState)
    {
      if (RCWLState == HIGH)
      {
        motionAlarms++;
        EnableAlarm();
        ledAlarmMotion.on();
        Blynk.virtualWrite(V9, motionAlarms);
      }
      RCWLLastState = RCWLState;
    }
  }

  if (timeAlarm > 2 && timeMillis - timeAlarm > 15000)
  {
    DisableAlarm();
  }
  
  else if (timeAlarm > 2)
  {
    BeepBuzzer(300);
    BlinkRed(300);
  }
  else if (timeAlarm == 0 && (soundAlarms > 0 || motionAlarms > 0 || tempHighAlarms > 0 || tempLowAlarms > 0 || gasAlarms > 0))
  {
    BlinkRed(1000);
  }
  else if (!guardEnabled)
  {
    if (timeLEDRed == 0 && digitalRead(pinRCWL) == HIGH)
    {
      digitalWrite(pinLEDRed, HIGH);
      timeLEDRed = 1;
      ledRed.on();
      ledAlarmMotion.on();
    }
    else if (timeLEDRed == 1 && digitalRead(pinRCWL) == LOW)
    {
      digitalWrite(pinLEDRed, LOW);
      timeLEDRed = 0;
      ledRed.off();
      ledAlarmMotion.off();
    }
    else if (timeLEDRed == 2)
    {
      digitalWrite(pinLEDRed, HIGH);
      timeLEDRed = timeMillis;
      ledRed.on();
      ledAlarmSound.on();
    }
    else if (timeLEDRed > 2 && timeMillis - timeLEDRed > 300)
    {
      digitalWrite(pinLEDRed, LOW);
      timeLEDRed = 0;
      ledRed.off();
      ledAlarmSound.off();
    }
  }

  if (timeLEDGreen > 0 && timeMillis - timeLEDGreen > 500)
  {
    digitalWrite(pinLEDGreen, LOW);
    timeLEDGreen = 0;
    ledGreen.off();
  }

  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    timeLEDGreen = timeMillis;
    digitalWrite(pinLEDGreen, HIGH);
    memcpy(&DEVID, &buf, buflen);
    ledGreen.on();
  }

  Blynk.run();
  timer.run();
}
