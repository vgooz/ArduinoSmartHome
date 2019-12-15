#include "DHT.h"
#include <Adafruit_BMP280.h>

#define DHTTYPE DHT11

#define pinLED          13
#define pinDHT          5
//#define pinRCWL         2     //Micro Wave Sensor
#define pinRCWLSET      7     //Micro Wave Sensor On/Off motion detecting
#define pinSound        2
#define pinBuzzer       12

unsigned long timeMeasuring = 0;
unsigned long timeBuzzer = 0;
unsigned long timeSound = 0;
unsigned long timeMotion = 0;
unsigned long timeLed = 0;
unsigned long timeLed0 = 0;

volatile bool soundDetected = 0;
volatile bool motionDetected = 0;

DHT dht(pinDHT, DHTTYPE);
Adafruit_BMP280 bmp; //I2C

String inputString = "";
String inputTopic = "";
String inputValue = "";
bool inputAvailable = false;  // whether the input is complete

void onSoundDetect()
{
  soundDetected = true;
}

void onMotionDetect()
{
  motionDetected = true;
}

void setup() {
  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinSound, INPUT);
  pinMode(pinDHT, INPUT);
  pinMode(pinRCWLSET, INPUT_PULLUP);

//  attachInterrupt(0, onMotionDetect, RISING);
  attachInterrupt(0, onSoundDetect, RISING);

  Serial.begin(115200);
  inputString.reserve(50);
  inputTopic.reserve(40);
  inputValue.reserve(10);
  
  dht.begin();
  bmp.begin(0x76);
}

void loop() {

  unsigned long timeMillis = millis();

  if (inputAvailable)
  {
    if (inputTopic == "board0/led/set")
    {
      if (inputValue == "ON") 
      {
        if (digitalRead(pinLED) == LOW)
        {
          digitalWrite(pinLED, HIGH);
          Serial.println("board0/led ON");
        }
        timeLed = timeMillis;
      }
      else 
      {
        if (digitalRead(pinLED) == HIGH)
        {
          digitalWrite(pinLED, LOW);
          Serial.println("board0/led OFF");
        }
        timeLed = 0;
      }
    }
    else if (inputTopic == "board0/beep/set") 
    {
      if (inputValue == "ON") 
      {
        if (digitalRead(pinBuzzer) == LOW)
        {
          digitalWrite(pinBuzzer, HIGH);
          Serial.println("board0/beep ON");
        }
        timeBuzzer = timeMillis;
      }
      else 
      {
        if (digitalRead(pinBuzzer) == HIGH)
        {
          digitalWrite(pinBuzzer, LOW);
          Serial.println("board0/beep OFF");
        }
        timeBuzzer = 0;
      }
    }
    else if (inputValue == "get_topics")
    {
      Serial.println("board0/led/set");
      Serial.println("board0/beep/set");
      Serial.println("end");
    }
    inputTopic = "";
    inputValue = "";
    inputAvailable = false;
  }
  
  if (motionDetected)
  {
    if (timeSound == 0)
    {
      Serial.println("board0/motion ON");
      digitalWrite(pinLED, HIGH);
      Serial.println("board0/led ON");
      timeMotion = timeMillis;
    }
    motionDetected = false;
  }
  if (timeMotion > 0 && timeMillis - timeMotion > 1000)
  {
    Serial.println("board0/motion OFF");
    digitalWrite(pinLED, LOW);
    Serial.println("board0/led OFF");
    timeMotion = 0;
  }
  
  if (soundDetected)
  {
    soundDetected = false;
    if (timeSound == 0)
    {
      Serial.println("board0/sound ON");
      digitalWrite(pinLED, HIGH);
      Serial.println("board0/led ON");
      timeSound = timeMillis;
    }
  }
  if (timeSound > 0 && timeMillis - timeSound > 1000)
  {
    Serial.println("board0/sound OFF");
    digitalWrite(pinLED, LOW);
    Serial.println("board0/led OFF");
    timeSound = 0;
  }

  if (timeLed > 0 && timeMillis - timeLed > 1000 && digitalRead(pinLED) == LOW)
  {
    digitalWrite(pinLED, HIGH);
    Serial.println("board0/led ON");
    timeLed = timeMillis;
  }
  if (timeLed > 0 && timeMillis - timeLed > 1000 && digitalRead(pinLED) == HIGH && timeSound == 0 && timeMotion == 0 && timeLed0 == 0)
  {
    digitalWrite(pinLED, LOW);
    Serial.println("board0/led OFF");
    timeLed = timeMillis;
  }
  
  if (timeBuzzer > 0 && timeMillis - timeBuzzer > 500 && digitalRead(pinBuzzer) == LOW)
  {
    digitalWrite(pinBuzzer, HIGH);
    Serial.println("board0/beep ON");
    timeBuzzer = timeMillis;  
  }
  if (timeBuzzer > 0 && timeMillis - timeBuzzer > 500 && digitalRead(pinBuzzer) == HIGH)
  {
    digitalWrite(pinBuzzer, LOW);
    Serial.println("board0/beep OFF");
    timeBuzzer = timeMillis;
  }

  if (timeMillis - timeMeasuring > 30000)
  {
    digitalWrite(pinLED, HIGH);
    Serial.println("board0/led ON");
    timeMeasuring = timeMillis;
    timeLed0 = timeMillis;;
    Serial.print("board0/dht11/temperature ");
    Serial.println(dht.readTemperature());
    Serial.print("board0/dht11/humidity ");
    Serial.println(dht.readHumidity());
    Serial.print("board0/bmp280/temperature ");
    Serial.println(bmp.readTemperature());
    Serial.print("board0/bmp280/pressure ");
    Serial.println(bmp.readPressure() / 100);
    motionDetected = false; //fix fail motion
  }
  if (timeLed0 > 0 && timeMillis - timeLed0 > 500)
  {
    digitalWrite(pinLED, LOW);
    Serial.println("board0/led OFF");
    timeLed0 = 0;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') 
    {
      inputAvailable = true;
      inputValue = inputString;
      inputString = "";
    }
    else if (inChar == ' ')
    {
      inputTopic = inputString;
      inputString = "";
    }
    else inputString += inChar;
  }
}
