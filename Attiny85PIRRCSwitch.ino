#include <avr/wdt.h>
#include <avr/sleep.h>
#include <RCSwitch.h>

#define aca_disable() (ACSR |= _BV(ACD))
#define adc_disable() (ADCSRA &= ~_BV(ADEN))
#define adc_enable()  (ADCSRA |=  _BV(ADEN))


#define LED_PIN 0     //pin for blinking LED
#define PIR_PIN 2
#define TX_PIN  4     //pin where your transmitter is connected

#define VCC_HIGH              4700  //4.7V
#define VCC_LOW               3100  //3.1V
#define VCC_OFF               2800  //2.8V

#define WAKEUP_INTERVAL       40     // * 8 sec

struct package {
  uint8_t group;
  uint8_t device: 4, sensor: 3, ecc: 1; //ecc - errors control check
  uint16_t value;
};

typedef struct package Package;
Package data;
uint8_t buf[sizeof(unsigned long)];

uint16_t lastVccValue = VCC_HIGH;

RCSwitch mySwitch = RCSwitch();

volatile bool motionDetected = false;
volatile byte watchdog_counter = WAKEUP_INTERVAL;

ISR(WDT_vect) {
  watchdog_counter++;
  if (motionDetected && watchdog_counter > 8) motionDetected = false;
}

ISR(PCINT0_vect) {
    motionDetected = true;
    watchdog_counter = WAKEUP_INTERVAL;
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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(3, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  mySwitch.enableTransmit(TX_PIN);
  data.group = 111;
  data.device = 2;
  data.ecc = 1;
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void loop() {

  if (watchdog_counter < WAKEUP_INTERVAL)
  {
    cli();                                  // Disable interrupts

    digitalWrite(LED_PIN, LOW);
    digitalWrite(1, LOW);
    digitalWrite(3, LOW);
    digitalWrite(TX_PIN, LOW);
    digitalWrite(5, LOW);
    
    //Initialize Watchdog Timer
    wdt_reset();
    wdt_enable(WDTO_8S);                    //sleep 8 sec
    WDTCR |= _BV(WDIE);                     //Enable watchdog interrupts
    if (!motionDetected)
    {    
      GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
      PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
    }
    sleep_bod_disable();
    aca_disable();
    adc_disable();
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();                                  // Enable interrupts
    sleep_cpu();

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin
    sleep_disable();
    adc_enable();
    sei();                                  // Enable interrupts
    
    return; // wait for WAKEUP_INTERVAL * 8 seconds
  }

  watchdog_counter = 0;

  if (lastVccValue > 0 && lastVccValue < VCC_OFF) return;

  digitalWrite(LED_PIN, HIGH);
    
  if (motionDetected)
  {
    digitalWrite(PIR_PIN, LOW);
    data.sensor = 2;
    data.value = 1;
  }
  else
  {
    lastVccValue = readVcc();
    data.sensor = 1;
    if (lastVccValue < VCC_HIGH)
      data.value = round((lastVccValue - VCC_OFF) * (100.0 / (VCC_HIGH - VCC_OFF)));
    else
      data.value = 100;
  }

    buf[0] = data.group;
    buf[1] = data.value & 0xff;
    buf[2] = (data.value >> 8) & 0xff;
    buf[3] = data.device | data.sensor << 4 | data.ecc << 7;

    unsigned long ulbuf = (((unsigned long)buf[0]) << 24) | (((unsigned long)buf[1]) << 16) | (((unsigned long)buf[2]) << 8) | (unsigned long)buf[3];

    mySwitch.send(ulbuf, 32);

    digitalWrite(LED_PIN, LOW);

  if (lastVccValue > 0 && lastVccValue < VCC_LOW) //Low Vcc Blink
  {
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }  
}
