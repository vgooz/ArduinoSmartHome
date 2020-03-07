#include <avr/wdt.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <RCSwitch.h>

#define aca_disable() (ACSR |= _BV(ACD))
#define adc_disable() (ADCSRA &= ~_BV(ADEN))
#define adc_enable()  (ADCSRA |=  _BV(ADEN))

#define TX_PIN  4     //pin where your transmitter is connected
#define LED_PIN 1     //pin for blinking LED

#define AM2320_CMD_READREG    0x03  ///< read register command
#define AM2320_REG_TEMP_H     0x02  ///< temp register address
#define AM2320_REG_HUM_H      0x00  ///< humidity register address
#define AM2320_I2C_ADDR       0x5C

#define VCC_HIGH              4200  //4.2V
#define VCC_LOW               2700  //2.6V
#define VCC_OFF               2500

#define WAKEUP_INTERVAL       10     // * 8 sec

struct package {
  uint8_t group;
  uint8_t device: 4, sensor: 3, ecc: 1; //ecc - errors control check
  uint16_t value;
};

typedef struct package Package;
Package data;
uint8_t buf[sizeof(unsigned long)];

uint16_t lastVccValue = VCC_HIGH;
byte i = 3;

RCSwitch mySwitch = RCSwitch();

volatile byte watchdog_counter = WAKEUP_INTERVAL;

ISR(WDT_vect) {
  watchdog_counter++;
}

uint16_t readVcc(void)
{
  uint16_t result;
  // Read 1.1V reference against Vcc
  ADMUX = (0 << REFS0) | (12 << MUX0);
  delay(2); // Wait for Vref to settle
  ADCSRA |= (1 << ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCW;
  return 1125300L / result; // Back-calculate AVcc in mV
}

uint16_t am2320_crc16(uint8_t *buffer, uint8_t nbytes) {
  uint16_t crc = 0xffff;
  for (int i = 0; i < nbytes; i++) {
    uint8_t b = buffer[i];
    crc ^= b;
    for (int x = 0; x < 8; x++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

uint16_t am2320_readRegister16(uint8_t reg) {
  // wake up
  Wire.beginTransmission(AM2320_I2C_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10); // wait 10 ms
  Wire.beginTransmission(AM2320_I2C_ADDR);
  Wire.endTransmission();

  // send a command to read register
  Wire.beginTransmission(AM2320_I2C_ADDR);
  Wire.write(AM2320_CMD_READREG);
  Wire.write(reg);
  Wire.write(2);  // 2 bytes
  Wire.endTransmission();

  delay(2);  // wait 2 ms

  // 2 bytes preamble, 2 bytes data, 2 bytes CRC
  Wire.requestFrom(AM2320_I2C_ADDR, (uint8_t)6);
  if (Wire.available() != 6)
    return 0xFFFF;

  uint8_t buffer[6];
  for (int i = 0; i < 6; i++) {
    buffer[i] = Wire.read();
  }

  if (buffer[0] != 0x03)   return 0xFFFF; // must be 0x03 modbus reply
  if (buffer[1] != 2)      return 0xFFFF; // must be 2 bytes reply

  uint16_t the_crc = buffer[5];
  the_crc <<= 8;
  the_crc |= buffer[4];
  uint16_t calc_crc = am2320_crc16(buffer, 4); // preamble + data
  //Serial.print("CRC: 0x"); Serial.println(calc_crc, HEX);
  if (the_crc != calc_crc)
    return 0xFFFF;

  // All good!
  uint16_t ret = buffer[2];
  ret <<= 8;
  ret |= buffer[3];

  return ret;
}

void setup() {
  pinMode(0, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  mySwitch.enableTransmit(TX_PIN);
  Wire.begin();
  data.group = 111;
  data.device = 1;
  data.ecc = 1;
  delay(500);
  delay(500);
  digitalWrite(LED_PIN, LOW);  
}

void loop() {
  
  if (watchdog_counter < WAKEUP_INTERVAL)
  {
    cli();                // Disable interrupts

    digitalWrite(0, LOW);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(TX_PIN, LOW);
    digitalWrite(5, LOW);

    //Initialize Watchdog Timer
    wdt_reset();
    wdt_enable(WDTO_8S);    //sleep 8 sec
    WDTCR |= _BV(WDIE);     //Enable watchdog interrupts

    sleep_bod_disable();
    aca_disable();
    adc_disable();
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();                // Enable interrupts

    sleep_cpu();

    cli();                                  // Disable interrupts
    sleep_disable();
    adc_enable();
    sei();                                  // Enable interrupts
    
    return; // wait for 2 * 8 seconds
  }

  watchdog_counter = 0;

  if (lastVccValue > 0 && lastVccValue < VCC_OFF) return;
  
  digitalWrite(LED_PIN, HIGH);
  
  if (i > 9) i = 0;

  if (i == 3)
  {
    lastVccValue = readVcc();
    data.sensor = 1;
    if (lastVccValue < VCC_HIGH) 
      data.value = round((lastVccValue - VCC_OFF) * (100.0 / (VCC_HIGH - VCC_OFF)));
    else
      data.value = 100;    
  }
  else if (i == 1 || i == 5 || i == 8)
  {
    data.sensor = 4;
    data.value = round((float)am2320_readRegister16(AM2320_REG_HUM_H)/10);
  }
  else
  {
    data.sensor = 3;
    data.value = am2320_readRegister16(AM2320_REG_TEMP_H);    
  }
  i++;
  
  buf[0] = data.group;
  buf[1] = data.value & 0xff;
  buf[2] = (data.value >> 8) & 0xff;
  buf[3] = data.device | data.sensor << 4 | data.ecc << 7;

  unsigned long ulbuf = (((unsigned long)buf[0]) << 24) | (((unsigned long)buf[1]) << 16) | (((unsigned long)buf[2]) << 8) | (unsigned long)buf[3];

  mySwitch.send(ulbuf, 32);

  digitalWrite(LED_PIN, LOW);
  
  if (lastVccValue < VCC_LOW) //Low Vcc Blink
  {
    delay(100);    
    PORTB |= (1 << PB1);      //replaces digitalWrite(pinLED, HIGH);
    delay(200);
    PORTB &= ~(1 << PB1);    //replaces digitalWrite(pinLED, LOW);
    delay(100);
    PORTB |= (1 << PB1);      //replaces digitalWrite(pinLED, HIGH);
    delay(200);
    PORTB &= ~(1 << PB1);    //replaces digitalWrite(pinLED, LOW);
    delay(100);
    PORTB |= (1 << PB1);      //replaces digitalWrite(pinLED, HIGH);
    delay(200);
  }
}