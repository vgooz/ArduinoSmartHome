#include <ESP8266WiFi.h>
#include <RCSwitch.h>
#include <PubSubClient.h>

#define LED_PIN         D4
#define RX_PIN          D5

#define WLAN_SSID       "GooZz2.4GHz"
#define WLAN_PASS       "bluegene54321"
#define MQTT_USER       "mqttusr"
#define MQTT_PASS       "mq123tt"

struct package
{
  uint8_t group;
  uint8_t device: 4, sensor: 3, ecc: 1; //ecc - errors control check
  uint16_t value;
};

typedef struct package Package;
Package data;
uint8_t buf[sizeof(unsigned long)];

WiFiClient wifiClient;
IPAddress brocker_ip(192,168,1,100);
PubSubClient mqttClient(brocker_ip, 1883, wifiClient);
RCSwitch mySwitch = RCSwitch();

unsigned long timeRX = 0;
uint8_t device = 0;
uint16_t battery = 0, temperature = 0, humidity = 0;

bool nochanges = false;
unsigned long deviceTime = 0;

void connectToMQTT()
{
  Serial.print("Connecting to the MQTT broker ");
  Serial.println(brocker_ip);
  while (!mqttClient.connected()) {
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println("MQTT Brocker connected.");    
    }
    else {
      Serial.print("Connection Failed, Err=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.print("Connecting to the WiFi ");
  Serial.print(WLAN_SSID);
  
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println();
  Serial.print("WiFi connected with IP address ");
  Serial.println(WiFi.localIP());

  connectToMQTT();
  
  mySwitch.enableReceive(RX_PIN);  // Receiver on interrupt 0 => that is pin #2
  mySwitch.setReceiveTolerance(80);

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
      Serial.print(F("Received Unknown Package "));
      Serial.print(p);
      Serial.print(F("/"));
      Serial.print(l);      
      Serial.print(F(" "));
      Serial.println(ulbuf);
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
        mqttClient.publish("bridge0/led", "ON");
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

  if (timeRX > 0 && timeMillis - timeRX < 1500) return;

  if (timeRX == 0) 
  {
    if (!mqttClient.connected()) {
      digitalWrite(LED_PIN, LOW);
      connectToMQTT();
      digitalWrite(LED_PIN, HIGH);
    }
    mqttClient.loop();
    return;
  }

  pushDataToMQTT();
  mqttClient.publish("bridge0/led", "OFF");
  timeRX = 0;
  digitalWrite(LED_PIN, HIGH);
}

void pushDataToMQTT()
{
  if (device == 0) return;
  
  Serial.print("Received Package from Device ");
  Serial.print(device);
  
  if (nochanges)
  {
    Serial.println(" No Changes");
    nochanges = false;
    device = 0;
    return;
  }

  String topic = "bridge0/device" + String(device);
  
  if (battery > 0)
  {
    Serial.print(" Battery: ");
    Serial.print(battery);
    Serial.print(" mV");
    topic += "/battery";
    mqttClient.publish(topic.c_str(), String(battery).c_str());
    battery = 0;
  }
  
  if (temperature > 0)
  {
    float t = temperature & 0x8000 ? -(float)(temperature & 0x7fff) / 10 : (float)temperature / 10;
    Serial.print(" Temperature: ");
    Serial.print(t);
    Serial.print("\xC2\xB0");
    Serial.print("C");
    topic += "/temperature";
    mqttClient.publish(topic.c_str(), String(t).c_str());
    temperature = 0;
  }
  
  if (humidity > 0)
  {
    Serial.print(" Humidity: ");
    Serial.print(humidity);
    Serial.print("%");
    topic += "/humidity";
    mqttClient.publish(topic.c_str(), String(humidity).c_str());
    humidity = 0;
  }
  Serial.println();
  device = 0;
}
