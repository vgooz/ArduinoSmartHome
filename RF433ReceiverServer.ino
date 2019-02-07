//P0(SDA), P2(SCL)

#include <avr/wdt.h>
#include <VirtualWire.h>
#include <TinyWireS.h>

#define pinLED    1
#define pinRX     3
#define pinRXINT  4
#define repeatTX  15
#define I2C_ADDR  9
#define rxBufLen  3
#define i2cBufLen 4

#define RX_SPEED  2048

uint8_t prevSender = 0;

uint8_t bufI2C[i2cBufLen];

void setup() {
  DDRB |= (1 << PB1);      //replaces pinMode(pinLED, OUTPUT);
  DDRB |= (1 << PB4);      //replaces pinMode(pinRXIN, OUTPUT);
  initializeWatchdogTimer(WDTO_2S);
  vw_set_rx_pin(pinRX);
  vw_setup(RX_SPEED);
  vw_rx_start();
  TinyWireS.begin(I2C_ADDR);
  TinyWireS.onRequest(requestEvent);
}

ISR(WDT_vect) {
  PORTB &= ~(1 << PB1);    //replaces digitalWrite(pinLED, LOW);
  PORTB &= ~(1 << PB4);    //replaces digitalWrite(pinLED, LOW);
}

void initializeWatchdogTimer(byte sleep_time)
{
  wdt_reset();
  wdt_enable(sleep_time);
}

void requestEvent()
{
  for (int i = 0; i < i2cBufLen; i++) TinyWireS.send(bufI2C[i]);
}

void loop()
{
  vw_wait_rx();

  uint8_t buflen = rxBufLen;
  uint8_t buf[rxBufLen];

  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    if (buf[0] != prevSender)
    {
      PORTB |= (1 << PB1);      //replaces digitalWrite(pinLED, HIGH);
      PORTB |= (1 << PB4);      //replaces digitalWrite(pinRXINT, HIGH);
      WDTCR |= _BV(WDIE);       //Enable watchdog interrupts
      bufI2C[0] = buf[0];       //sender
      bufI2C[1] = 0;            //packages received
      bufI2C[2] = buf[1];       //package data value first byte
      bufI2C[3] = buf[2];       //packages data value last byte
      prevSender = buf[0];
    }
    bufI2C[1]++;
  }
}
