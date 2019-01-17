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
  uint8_t sender;
  uint8_t session;
  uint8_t iteration;
  uint8_t received;
  uint8_t lost;
  int16_t data1;
  int16_t data2;
  int16_t data3;
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
  
  if (timeLEDGreen > 0 && timeMillis - timeLEDGreen > 500)
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
        CopyBufferToPackage(buf);
        Serial.print("Received package! buflen: ");
        Serial.print(buflen);
        Serial.print("; bytes: ");
        Serial.print(bytes);
        Serial.print("; Sender: ");
        Serial.print(data.sender);
        Serial.print("; Session: ");
        Serial.print(data.session);
        Serial.print("; Iteration: ");
        Serial.print(data.iteration);
        Serial.print("; Received: ");
        Serial.print(data.received);
        Serial.print("; Lost: ");
        Serial.print(data.lost);
        Serial.print("; Data1: ");
        Serial.print(data.data1);
        Serial.print("; Data2: ");
        Serial.print(data.data2);
        Serial.print("; Data3: ");
        Serial.print(data.data3);
        Serial.print("; Quality: ");
        if (data.received + data.lost > 0)
          Serial.print(((float)data.received / (float)(data.received + data.lost)) * 100.0);
        else
          Serial.print("0.00");
        Serial.println("%");
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

void CopyBufferToPackage(uint8_t *arr)
{
  data.sender = arr[0];   
  data.session = arr[1];
  data.iteration = arr[2];
  data.received = arr[3];
  data.lost = arr[4];
  data.data1 = arr[5] | arr[6] << 8;
  data.data2 = arr[7] | arr[8] << 8;
  data.data3 = arr[9] | arr[10] << 8;
}
