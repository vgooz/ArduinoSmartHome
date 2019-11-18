#include <RCSwitch.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WidgetRTC.h>

#define LED_PIN         D4
#define RX_PIN          D5

struct package
{
  uint8_t group;
  uint8_t device: 4, sensor: 3, ecc: 1; //ecc - errors control check
  uint16_t value;
};

typedef struct package Package;
Package data;
uint8_t buf[sizeof(unsigned long)];

RCSwitch mySwitch = RCSwitch();

unsigned long timeRX = 0;
uint8_t device = 0;
uint16_t battery = 0, temperature = 0, humidity = 0;

bool nochanges = false;

char auth[] = "807cd2f8cb1f41ad92398704330b4e3e";
char ssid[] = "GooZz2.4GHz";
char pass[] = "bluegene54321";

#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
#define BLYNK_DARK_BLUE "#5F7CD8"

//V0 - LED Package receiving, V1 - LCD, V2 - Temperature, V3 - Humidity, V4 - Device 1 Battery Level, V5 - Device 1 LED infarred, V6 - Terminal
WidgetLED ledGreen(V0);     //LED Green
WidgetLCD lcd (V1);         //LCD
WidgetTerminal terminal(V6);
WidgetRTC rtc;

unsigned long deviceTime = 0;

BlynkTimer pushTimer;

BLYNK_CONNECTED()
{
  rtc.begin();
  if (timeRX > 0) ledGreen.on();
  else ledGreen.off();

  lcd.clear();
  lcd.print(0, 0, "D");
  lcd.print(9, 0, "mV");
  lcd.print(9, 1, "\xC2\xB0");
  lcd.print(10, 1, "C");
  lcd.print(15, 1, "%");
}

void setup() {
  
  pinMode(LED_PIN, OUTPUT);
  
  mySwitch.enableReceive(RX_PIN);  // Receiver on interrupt 0 => that is pin #2
  mySwitch.setReceiveTolerance(80);

  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass, IPAddress(192, 168, 1, 100), 8181);
  pushTimer.setInterval(1000L, pushData);

  digitalWrite(LED_PIN, HIGH);
}

void loop() {

  unsigned long timeMillis = millis();

  if (mySwitch.available())
  {
    unsigned int p = mySwitch.getReceivedProtocol();
    unsigned int l = mySwitch.getReceivedBitlength();
    unsigned long ulbuf = mySwitch.getReceivedValue();

    data.ecc=0;
    bool isKnown = p == 1 && l == 32;

    if (isKnown)
    {
      buf[0] = (int)((ulbuf >> 24) & 0xFF) ;
      isKnown = buf[0] == 111;
    }
    
    if (!isKnown)
    {
      digitalWrite(LED_PIN, LOW);
      ledGreen.on();
      terminal.print(day());
      terminal.print(F("/"));
      terminal.print(month());
      terminal.print(F(" "));
      terminal.print(hour());
      terminal.print(F(":"));
      terminal.print(minute());
      terminal.print(F(" "));
      terminal.print(F("Unknown "));
      terminal.print(p);
      terminal.print(F("/"));
      terminal.print(l);      
      terminal.print(F(" "));
      terminal.println(ulbuf);
      terminal.flush();
      ledGreen.off();
      digitalWrite(LED_PIN, HIGH);
    }
    
    if (isKnown)
    {
      buf[0] = (int)((ulbuf >> 24) & 0xFF) ;
      buf[1] = (int)((ulbuf >> 16) & 0xFF) ;
      buf[2] = (int)((ulbuf >> 8) & 0XFF);
      buf[3] = (int)((ulbuf & 0XFF));

      data.group = buf[0];
      data.value = buf[1] | buf[2] << 8;
      data.device = buf[3] & 0xf;
      data.sensor = (buf[3] >> 4) & 0x7;
      data.ecc = (buf[3] >> 7) & 0x1;
      if (timeRX == 0)
      {
        timeRX = timeMillis;
        digitalWrite(LED_PIN, LOW);
        ledGreen.on();
      }
    }
    if (data.ecc)
    {
      device = data.device;
      if (data.sensor == 1) battery = data.value;
      else if (data.sensor == 3) temperature = data.value;
      else if (data.sensor == 4) humidity = data.value;
      else nochanges = true;
      deviceTime = timeMillis;
    }
    mySwitch.resetAvailable();
  }

  if (timeRX > 0 && timeMillis - timeRX < 3000) return;

  if (timeRX == 0)
  {
    Blynk.run();
    pushTimer.run();
    return;
  }

  timeRX = 0;
  ledGreen.off();
  digitalWrite(LED_PIN, HIGH);
}

void pushData()
{
  
  if (device == 0) 
  {
    unsigned long t = (millis() - deviceTime) / 1000;
    
    if (t > 9999)
    {
      lcd.print(12, 0, "----");
      lcd.print(3, 0, "-");
      return;
    }
    
    if (t < 10)
    {
      lcd.print(12, 0, "   ");
      lcd.print(15, 0, t);
    }
    else if (t < 100)
    {
      lcd.print(12, 0, "  ");
      lcd.print(14, 0, t);      
    }
    else if (t < 1000)
    {
      lcd.print(12, 0, " ");
      lcd.print(13, 0, t);      
    }
    else if (t < 10000)
    {
      lcd.print(12, 0, t);      
    }
    return;
  }
  
  lcd.print(1, 0, device);
  lcd.print(12, 0, "   0");
  lcd.print(0, 1, "   ");
  
  device = 0; 
  
  if (nochanges)
  {
    nochanges = false;
    lcd.print(3, 0, "-");
    return;
  }
  
  lcd.print(3, 0, "*");

  if (battery > 0)
  {
    Blynk.virtualWrite(V4, battery);
    lcd.print(0, 1, "V");
    lcd.print(5, 0, battery);
    battery = 0;
  }
  
  if (temperature > 0)
  {
    float t = temperature & 0x8000 ? -(float)(temperature & 0x7fff) / 10 : (float)temperature / 10;
    Blynk.virtualWrite(V2, t);
    lcd.print(1, 1, "T");
    if (t > 0)
    {
      lcd.print(4, 1, " ");
      if (t < 10) 
      {
        lcd.print(5, 1, " ");
        lcd.print(6, 1, String(t, 1));
      }
      else if (t < 100) lcd.print(5, 1, String(t, 1));
      else lcd.print(4, 1, "ERROR"); 
    }
    else
    {
      if (t > -10) 
      {
        lcd.print(4, 1, " ");
        lcd.print(5, 1, String(t, 1));
      }
      else if (t > -100) lcd.print(4, 1, String(t, 1));
      else lcd.print(4, 1, "ERROR"); 
    }
    temperature = 0;
  }
  
  if (humidity > 0)
  {
    Blynk.virtualWrite(V3, humidity);
    lcd.print(2, 1, "H");
    if (humidity < 10) 
    {
      lcd.print(13, 1, " ");
      lcd.print(14, 1, humidity);
    }
    else if (humidity < 100) lcd.print(13, 1, humidity);
    else lcd.print(12, 1, "ERR");
    humidity = 0;
  }
}
