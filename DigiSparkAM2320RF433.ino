//AM2320 1(VDD), 2(SDA), 3(GND), 4(SCL)
//P0(SDA), P2(SCL)
#include <avr/sleep.h>
#include <avr/power.h>
#include <VirtualWire.h>
#include <Wire.h>

#define pinLED  1
#define pinTX   3

#define AM2320_CMD_READREG    0x03  ///< read register command
#define AM2320_REG_TEMP_H     0x02  ///< temp register address
#define AM2320_REG_HUM_H      0x00  ///< humidity register address
#define AM2320_I2C_ADDR       0x5C
#define TX_SPEED              2048

struct package
{
  uint8_t sender: 4, session: 2, type: 2;
  uint16_t value;
};

typedef struct package Package;
Package data;
uint8_t buflen = sizeof(data);
uint8_t buf[sizeof(data)];
uint8_t session;
uint8_t iteration;

volatile byte f_wdt = 1;

void setup()
{
  DDRB |= (1 << PB1);      //replaces pinMode(pinLED, OUTPUT);
  data.sender = 1;
  vw_set_tx_pin(pinTX);
  vw_setup(TX_SPEED);
  Wire.begin();

  // Setup the WDT
  MCUSR &= ~(1 << WDRF);                              // Clear the reset flag

  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).
  WDTCR |= (1 << WDCE) | (1 << WDE);
  WDTCR = 1 << WDP0 | 1 << WDP3;                      // set new watchdog timeout prescaler value 8.0 seconds
  WDTCR |= _BV(WDIE);                                 // Enable the WD interrupt (note no reset).
}

void loop()
{
  static int awake_counter = 1;
  if (f_wdt == 1) {
    f_wdt = 0;
    --awake_counter;
    if (awake_counter <= 0) {                         // Send Weather data
      awake_counter = 5;                              //40 Sec

      PORTB |= (1 << PB1);      //replaces digitalWrite(pinLED, HIGH);
      
      iteration = 0;
      if (session % 5 == 0)
      {
        data.type = 0;
        data.value = (float)readVcc();
      }
      else if (session % 3 == 0)
      {
          data.type = 2;
          data.value = am2320_readRegister16(AM2320_REG_HUM_H);
      }
      else
      {
          data.type = 1;
          data.value = am2320_readRegister16(AM2320_REG_TEMP_H);
      }
            
      buf[0] = data.sender | data.session << 4 | data.type << 6;
      buf[1] = data.value & 0xff;
      buf[2] = (data.value >> 8) & 0xff;

      while(iteration < 33)
      {
        vw_send((uint8_t *)&buf, buflen);
        vw_wait_tx();
        iteration++;
        delay(5);
      }

      data.session++;
      data.type++;

      session++;

      PORTB &= ~(1 << PB1);    //replaces digitalWrite(pinLED, LOW);
    }
    enterSleep();
  }
}

ISR(WDT_vect) {
  if (f_wdt == 0) f_wdt = 1;
}

void enterSleep(void) {
  power_all_disable();
  ADCSRA &= ~(1 << ADEN);                             // disable ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                // the lowest power consumption
  sleep_enable();
  sleep_cpu();                                        // Now enter sleep mode.

  // The program will continue from here after the WDT timeout.
  sleep_disable();                                    // First thing to do is disable sleep.
  power_all_enable();                                 // Re-enable the peripherals.
  ADCSRA |= 1 << ADEN;                                // enable ADC
}

uint16_t readVcc(void) 
{
  uint16_t result;
  // Read 1.1V reference against Vcc
  ADMUX = (0<<REFS0) | (12<<MUX0);
  delay(2); // Wait for Vref to settle
  ADCSRA |= (1<<ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCW;
  return 1125300L / result; // Back-calculate AVcc in mV
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
    //Serial.print("byte #"); Serial.print(i); Serial.print(" = 0x"); Serial.println(buffer[i], HEX);
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
