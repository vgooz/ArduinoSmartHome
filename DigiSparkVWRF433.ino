#include <VirtualWire.h>
//#include <DigiKeyboard.h>

#define pinLED  1
#define pinTX   3

struct package
{
  int sender;
  int id;
  float data1;
  float data2;
  float data3;
};

typedef struct package Package;
Package data;

void setup()
{
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);
  data.sender = 1111;
  data.id = 0;
  vw_set_tx_pin(pinTX);
  vw_setup(300);
//  DigiKeyboard.println("Start...");
}

void loop()
{
  digitalWrite(pinLED, HIGH);
 
  data.id++;
  data.data1 = 23.0;
  data.data2 = 43.0;
//  DigiKeyboard.println((int)data.data1);
  vw_send((uint8_t *)&data, sizeof(data));
  vw_wait_tx();
  
  digitalWrite(pinLED, LOW);

  delay(2000);
}
