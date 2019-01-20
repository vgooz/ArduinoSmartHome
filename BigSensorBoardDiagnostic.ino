#include "DHT.h"
#include <Adafruit_BMP280.h>
#include <MQ135.h>
#include <Wire.h>

#define DHTTYPE DHT11

#define pinRX           2
#define DHTPIN          6
#define pinDMQ135       4     //Gas Sensor
#define pinRCWL         5     //Micro Wave Sensor
#define pinMQ135        A6
#define pinLEDGreen     12
#define pinLEDRed       13
#define pinBuzzer       7
#define pinBuzzerOn     8
#define I2C_ADDR        9

int RCWLLastState = LOW;
int DMQ135LastState = HIGH;

unsigned long timeMeasuring = 0;
unsigned long timeLEDRed = 0;
unsigned long timeLEDGreen = 0;
unsigned long timeBuzzer = 0;

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp; //I2C
MQ135 gasSensor = MQ135(pinMQ135);

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

void onSoundDetect()
{
  digitalWrite(pinLEDRed, HIGH);
  timeLEDRed = millis();
  Serial.println("Sound Detected!");
}

void onPackageReceived() {
  timeRX = millis();
}

void setup() {
  pinMode(pinLEDRed, OUTPUT);
  pinMode(pinLEDGreen, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinBuzzerOn, INPUT_PULLUP);

  dht.begin();
  bmp.begin(0x76);
  Wire.begin();
  attachInterrupt(0, onPackageReceived, RISING);  
  attachInterrupt(1, onSoundDetect, RISING);
  Serial.begin(9600);
}

void loop() {

  unsigned long timeMillis = millis();
  
  int RCWLState = digitalRead(pinRCWL);
  if (RCWLState != RCWLLastState)
  {
    if (RCWLState == HIGH) 
    {
      digitalWrite(pinLEDRed, HIGH);
      timeLEDRed = timeMillis;
      digitalWrite(pinBuzzer, HIGH);
      timeBuzzer = timeMillis;
      Serial.println("Motion Detected!");
    }
    RCWLLastState = RCWLState;
  }

  int DMQ135State = digitalRead(pinDMQ135);
  if (DMQ135State != DMQ135LastState)
  {
    if (DMQ135State == LOW)
    {
      digitalWrite(pinLEDRed, HIGH);
      timeLEDRed = timeMillis;
//      digitalWrite(pinBuzzer, HIGH);
      timeBuzzer = timeMillis;
      Serial.println("Gas Detected!");
    }
    DMQ135LastState = DMQ135State;
  }

  if (timeLEDRed > 0 && timeMillis - timeLEDRed > 500)
  {
    timeLEDRed = 0;
    digitalWrite(pinLEDRed, LOW);
  }

  if (timeBuzzer > 0 && timeMillis - timeBuzzer > 500)
  {
    timeBuzzer = 0;
    digitalWrite(pinBuzzer, LOW);
  }
  
  if (timeLEDGreen > 0 && timeMillis - timeLEDGreen > 1000)
  {
    timeLEDGreen = 0;
    digitalWrite(pinLEDGreen, LOW);
  }
  
  if (timeRX > 0 && timeMillis - timeRX > 3000)
  {
    timeRX = 0;
    Serial.println("Package Interrupt!");
    digitalWrite(pinLEDGreen, HIGH);
    timeLEDGreen = timeMillis;

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

        Serial.print("Received package! buflen: ");
        Serial.print(buflen);
        Serial.print("; bytes: ");
        Serial.print(bytes);
        Serial.print("; Sender: ");
        Serial.print(data.sender);
        Serial.print("; Session: ");
        Serial.print(data.session);
        Serial.print("; Received: ");
        Serial.print(data.received);
        Serial.print("; Data");
        Serial.print(data.type);
        Serial.print(": ");
        Serial.print(data.value);
        Serial.print("; Quality: ");
        Serial.print((float)data.received / 33.0 * 100.0);
        Serial.println("%");
        if (data.sender == 1)
        {
          switch (data.type)
          {
            case 0:
              Serial.print("Sensor Voltage = ");
              Serial.print(data.value / 1000.0);
              Serial.println("V");
              break;
            case 1:
              Serial.print("Sensor t = ");
              if (data.value == 0xFFFF) Serial.print("NAN");
              else if (data.value & 0x8000) Serial.print(-(int16_t)(data.value & 0x7fff) / 10.0);
              else Serial.print(data.value / 10.0);
              Serial.println("*C");
              break;
            case 2:
              Serial.print("Sensor h = ");
              if (data.value == 0xFFFF) Serial.print("NAN");
              else Serial.print(data.value / 10.0);
              Serial.println("%");
              break;
          }
        }        
      }
      else
      {
        Serial.print("Package data is corrupted! Byte received: ");
        Serial.println(bytes);
      }
    }
    else
    {
      Serial.println("No Slave Device found!");
    }
  }
  
  if (timeMillis - timeMeasuring > 5000)
  {
    Serial.print("DHT11 t=");
    Serial.print(dht.readTemperature());
    Serial.print("; h=");
    Serial.print(dht.readHumidity());
    Serial.print("| BMP280 t=");
    Serial.print(bmp.readTemperature());
    Serial.print("; p=");
    Serial.print(bmp.readPressure());
    Serial.print("| MQ135 ppm=");
    Serial.println(gasSensor.getPPM());
    timeMeasuring = timeMillis;
  }
}
