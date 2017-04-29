/**
   Pin notes by Suovula, see also http://hlt.media.mit.edu/?p=1229

   DIP and SOIC have same pinout, however the SOIC chips are much cheaper, especially if you buy more than 5 at a time
   For nice breakout boards see https://github.com/rambo/attiny_boards

   Basically the arduino pin numbers map directly to the PORTB bit numbers.

  // I2C
  arduino pin 0 = not(OC1A) = PORTB <- _BV(0) = SOIC pin 5 (I2C SDA)
  arduino pin 2 =           = PORTB <- _BV(2) = SOIC pin 7 (I2C SCL)
  // Timer1 -> PWM
  arduino pin 1 =     OC1A  = PORTB <- _BV(1) = SOIC pin 6 (LED)
  arduino pin 3 = not(OC1B) = PORTB <- _BV(3) = SOIC pin 2 (Servo Pulse)
  arduino pin 4 =     OC1B  = PORTB <- _BV(4) = SOIC pin 3 (ADC Servo)
  arduino pin 5 =     RESET            _BC(5) = SOIC pin 1 (RESET)

  Digital 2 is analog (ADC channel) 1
  Digital 3 is analog (ADC channel) 3
  Digital 4 is analog (ADC channel) 2
  Digital 5 is analog (ADC channel) 0
*/

// Get this from https://github.com/rambo/TinyWire
#include <TinyWireS.h>
#include "protocol.h"
#include <avr/sleep.h>

// The default buffer size, Can't recall the scope of defines right now
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 32 )
#endif

#ifndef TWI_TX_BUFFER_SIZE
#define TWI_TX_BUFFER_SIZE ( 32 )
#endif

// Proof of Concept Configuration
//#define LED     1  // no LED in POC
#define SERVOPIN  1
#define SENSORPIN 0 //digital 5 is adc channel 0

//#define INTERNAL2V56_NO_CAP (6)

/* Prototype Configuration
  #define LED       1  // no LED in POC
  #define SERVOPIN  3
  #define SENSORPIN 2 //digital 4 is adc channel 2
*/


short pos = 0;
short Commandpos = 0;
short Position = 0;

//#define WEEPROM     47  //flag for which eeprom value needs to be written out

#define EEPROM_SIZE 64

short reg[EEPROM_SIZE];  // all configurable values for smartservo
byte i2c_reg;            // which register is subject for r/w

byte excessbyte;

union CShort {
  byte b[2];
  signed short val;
} cshort;


unsigned long timermillis = 0;


/**
   This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the
   send-buffer when using this callback

   Data request packet has following elements:
   [REGISTERBYTE] [LOWBYTE] [HIGHBYTE] [CHECKSUMBYTE]
*/
void requestEvent()
{
  cshort.val = reg[i2c_reg];

  byte checksum = ~(i2c_reg + cshort.b[0] + cshort.b[1]);

  TinyWireS.send(i2c_reg);
  TinyWireS.send(cshort.b[0]);
  TinyWireS.send(cshort.b[1]);
  TinyWireS.send(checksum);

}



/*
   The I2C data received -handler

   This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
   so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,

   Data Write packet has the following elements:
   [REGISTERBYTE] [CHECKSUMBYTE]  set registers for read operation next
   [REGISTERBYTE] [LOWBYTE] [HIGHBYTE] [CHECKSUMBYTE] write a register
*/
void receiveEvent(uint8_t howMany) {
  byte receivedRegister;
  byte receivedChecksum;
  byte calculatedChecksum;

  if (howMany == 2) {
    // this is a register address for a read op
    //set register for i2c r/w operation
    receivedRegister = TinyWireS.receive();
    receivedChecksum = TinyWireS.receive();
    calculatedChecksum = ~(receivedRegister);

    if (receivedChecksum == calculatedChecksum) {
      i2c_reg = receivedRegister;
      reg[VALUE1] = i2c_reg;
    }
    else {
      reg[ERRORFLAGS] |= 1 << ERRORCHECKSUM;
    }

    howMany -= 2;
  }

  if (howMany == 4) {
    // this is a write register operation
    receivedRegister = TinyWireS.receive();
    cshort.b[0] = TinyWireS.receive();
    cshort.b[1] = TinyWireS.receive();
    receivedChecksum = TinyWireS.receive();
    calculatedChecksum = ~(receivedRegister + cshort.b[0] + cshort.b[1]);

    if (receivedChecksum == calculatedChecksum) {
      i2c_reg = receivedRegister;
      reg[i2c_reg] = cshort.val;
      reg[VALUE1] = i2c_reg;
      reg[VALUE2] = cshort.val;

      if(i2c_reg < 32) {
        cli();
        EEPROM_writeRegister(i2c_reg, reg[i2c_reg]);
        sei();
      }
    }

    howMany -= 4;
  }


  if (howMany > 0) {
    excessbyte = TinyWireS.receive();
    reg[VALUE5]++;
    howMany--;
  }
}

void EEPROM_writeByte(unsigned char ucAddress, unsigned char ucData)
{
  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE))
    ;

  /* Set Programming mode */
  EECR = (0 << EEPM1) | (0 << EEPM0);
  /* Set up address and data registers */
  EEAR = ucAddress;
  EEDR = ucData;
  /* Write logical one to EEMPE */
  EECR |= (1 << EEMPE);
  /* Start eeprom write by setting EEPE */
  EECR |= (1 << EEPE);

  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE))
    ;

}

unsigned char EEPROM_readByte(unsigned char ucAddress)
{
  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE))
    ;

  /* Set up address register */
  EEAR = ucAddress;
  /* Start eeprom read by writing EERE */
  EECR |= (1 << EERE);
  /* Return data from data register */

  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE))
    ;


  return EEDR;
}

void EEPROM_writeRegister(byte reg, short value) {
  byte address = reg * 2;
  cshort.val = value;
  cli();
  EEPROM_writeByte((address),     cshort.b[0]);
  EEPROM_writeByte((address + 1), cshort.b[1]);
  sei();
}

short EEPROM_readRegister(byte reg) {
  byte address = reg * 2;  // all reg contents are 16 bit shorts
  cli();
  cshort.b[0] = EEPROM_readByte(address);
  cshort.b[1] = EEPROM_readByte(address + 1);
  sei();
  return cshort.val;
}

void loadRegisters2() {

  reg[ID]         = EEPROM_readRegister(ID);
  reg[MINGOAL]    = EEPROM_readRegister(MINGOAL);
  reg[MAXGOAL]    = EEPROM_readRegister(MAXGOAL);
  reg[MAXSPEED]   = EEPROM_readRegister(MAXSPEED);
  reg[MINPULSE]   = EEPROM_readRegister(MINPULSE);
  reg[MAXPULSE]   = EEPROM_readRegister(MAXPULSE);
  reg[MINSENSOR]  = EEPROM_readRegister(MINSENSOR);
  reg[MAXSENSOR]  = EEPROM_readRegister(MAXSENSOR);
  reg[HEARTBEAT]  = EEPROM_readRegister(HEARTBEAT);
  reg[MAXTEMP]    = EEPROM_readRegister(MAXTEMP);

  reg[ENABLED]    = 0;
  //reg[GOAL]       = EEPROM_readRegister(ID);
  reg[PULSE]      = 1500;        // initial value to center
}

void loadRegisters() {

  reg[ID]         = 8;
  EEPROM_writeRegister(ID,reg[ID]);
  reg[MINGOAL]    = 0 * 100;
  reg[MAXGOAL]    = 180 * 100;
  reg[MAXSPEED]   = 0;
  reg[MINPULSE]   = 550;
  reg[MAXPULSE]   = 2550;
  reg[MINSENSOR]  = 808;
  reg[MAXSENSOR]  = 4700;
  reg[HEARTBEAT]  = B10100000;
  reg[MAXTEMP]    = 40;

  reg[ENABLED]    = 0;
  reg[GOAL]       = 90;
  reg[PULSE]      = 1500;        // initial value to center
}


/*
    The ISR is called 500 times a second at 8mhz
      So, 60 averaged samples 5 samples per iteration
      results in ~40 updates per second
*/
#define MAX_SAMPLES 60
#define BATCH_SAMPLES 5

byte sampleCount = 0;
unsigned long int sampleBucket = 0;
//long int samplestart = 0;

void updatePos() {
  // based on code from
  // http://21stdigitalhome.blogspot.com/2014/10/trinket-attiny85-internal-temperature.html

  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE)); // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN);                // Enable AD and start conversion
  ADMUX = SENSORPIN | _BV( REFS2 ) | _BV( REFS1 );         // ADC0 (PB4) and Ref voltage = 2.56V without external bypass capacitor;
  //delay(100);                         // Settling time min 1 ms, take 100 ms
  getADC();                         //discard first measurement

  for (int i = 0; i < BATCH_SAMPLES; i++) {
    sampleBucket += getADC();
  }

  ADCSRA &= ~(_BV(ADEN));        // disable ADC

  sampleCount += BATCH_SAMPLES;

  if (sampleCount >= MAX_SAMPLES) {
    sampleBucket /= (MAX_SAMPLES / 4);    // give an extra bit of subsampled resolution
    reg[RAWPOSITION] = sampleBucket;
    reg[POSITION] = map(sampleBucket, reg[MINSENSOR], reg[MAXSENSOR], reg[MINGOAL], reg[MAXGOAL]);
    sampleBucket = 0.0f;
    sampleCount = 0;
  }

}


short readTemp() {
  //http://21stdigitalhome.blogspot.com/2014/10/trinket-attiny85-internal-temperature.html

  // Setup the Analog to Digital Converter -  one ADC conversion
  //   is read and discarded
  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE)); // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN);                // Enable AD and start conversion
  ADMUX = 0xF | _BV( REFS1 );         // ADC4 (Temp Sensor) and Ref voltage = 1.1V;
  //delay(100);                         // Settling time min 1 ms, take 100 ms
  getADC();                         //discard first measurement
  // Measure temperature
  ADCSRA |= _BV(ADEN);           // Enable AD and start conversion
  ADMUX = 0xF | _BV( REFS1 );    // ADC4 (Temp Sensor) and Ref voltage = 1.1V;
  //delay(100);                    // Settling time min 1 ms, wait 100 ms

  short rawTemp = getADC();     // use next sample as initial average
  //for (short i=2; i<2000; i++) {   // calculate running average for 2000 measurements
  //  rawTemp += ((float)getADC() - rawTemp) / float(i);
  //}
  ADCSRA &= ~(_BV(ADEN));        // disable ADC

  rawTemp -= 273;               // previous adjustment to celcius code

  return rawTemp;

}



short readVcc_bad() {
  // based on code from
  // http://21stdigitalhome.blogspot.com/2014/10/trinket-attiny85-internal-temperature.html

  // Measure chip voltage (Vcc)
  ADCSRA |= _BV(ADEN);   // Enable ADC
  ADMUX  = 0x0c | _BV(REFS2);    // Use Vcc as voltage reference,
  //    bandgap reference as ADC input
  //delay(100);                    // Settling time min 1 ms, there is
  //    time so wait 100 ms
  long int rawVcc = getADC();      // discard first read

  ADCSRA |= _BV(ADEN);   // Enable ADC
  ADMUX  = 0x0c | _BV(REFS2);    // Use Vcc as voltage reference,
  //    bandgap reference as ADC input
  //delay(100);                    // Settling time min 1 ms, there is
  //    time so wait 100
  rawVcc = 0;
  for (byte i = 2; i < BATCH_SAMPLES; i++) { // calculate running average for 2000 measurements
    rawVcc += getADC();
  }
  ADCSRA &= ~(_BV(ADEN));        // disable ADC

  rawVcc /= BATCH_SAMPLES;

  //rawVcc = (1024 * (110)) / rawVcc;  // 1.1v * 100 for integer math

  return (short)rawVcc;


}


uint16_t readVcc(void) {
  // http://www.avrfreaks.net/comment/947156#comment-947156

  uint16_t result = 0;
  // Read 1.1V reference against Vcc
  ADMUX = (0 << REFS0) | (12 << MUX0);
  //delay(2); // Wait for Vref to settle
  //ADCSRA |= (1 << ADSC); // Convert
  //while (bit_is_set(ADCSRA, ADSC));
  //result = ADCW;
  result = getADC();
  result = 0;   //discard first read

  for (byte i = 0; i < BATCH_SAMPLES; i++) {
    result += getADC();
    //ADCSRA |= (1 << ADSC); // Convert
    //while (bit_is_set(ADCSRA, ADSC));
    //result += ADCW;
  }
  result /= BATCH_SAMPLES;
  //return 1125300L / result; // Back-calculate AVcc in mV
  return 112530L / result; // Back-calculate AVcc in integer V * 100
}


short getADC() {
  // Common code for all sources of an ADC conversion
  ADCSRA  |= _BV(ADSC);          // Start conversion
  while ((ADCSRA & _BV(ADSC)));   // Wait until conversion is finished
  return ADC;
}

// DOES NOT WORK, BOTH DURING ISR (LOCKS) AND IN LOOP (1 SECOND UPDATES)
short getQuietADC(byte pin) {
  ADMUX = bit (REFS0) | (pin & 0x07);

  // start the conversion
  ADCSRA |= bit (ADSC) | bit (ADIE);

  set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
  sleep_mode ();

  byte low, high;

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low = ADCL;
  high = ADCH;

  return (high << 8) | low;

}


void setup()
{
  pinMode(SERVOPIN, OUTPUT);
  digitalWrite(SERVOPIN, LOW);
  pinMode(SENSORPIN, INPUT);

  loadRegisters2();

  TinyWireS.begin(reg[ID]);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);

  //analogReference( INTERNAL2V56_NO_CAP );

  // Set up the interrupt that will refresh the servo for us automagically
  OCR0A = 0xAF;            // any number is OK
  TIMSK |= _BV(OCIE0A);    // Turn on the compare interrupt (below!)

  //initialize timermillis
  timermillis = millis();
}



void loop()
{
  /*
     This is the only way we can detect stop condition (http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=984716&sid=82e9dc7299a8243b86cf7969dd41b5b5#984716)
     it needs to be called in a very tight loop in order not to miss any (REMINDER: Do *not* use delay() anywhere, use tws_delay() instead).
     It will call the function registered via TinyWireS.onReceive(); if there is data in the buffer on stop.
  */
  TinyWireS_stop_check();



}

// Solution based heavily on Adafruit Trinket SoftServo
// https://github.com/adafruit/Adafruit_SoftServo

// We'll take advantage of the built in millis() timer that goes off
// to keep track of time, and refresh the servo every 20 milliseconds

// I think, seems right, need to look into timer settings
//#define INTERVAL 20  // for 16mhz clock?
#define INTERVAL 10    // for 8mhz clock?
volatile uint8_t counter = 0;

SIGNAL(TIMER0_COMPA_vect) {
  // this gets called every 2 milliseconds
  counter += 1;
  // every 20 milliseconds, refresh the servos!
  if (counter >= INTERVAL) {
    counter = 0;
    if (reg[ENABLED]) {
      digitalWrite(SERVOPIN, HIGH);
      delayMicroseconds(reg[PULSE]);
      digitalWrite(SERVOPIN, LOW);
    }

    /*
       Things you want to do after the servo pulse, 50 times a second
    */

    //otherwise we can have goals outside the max and min
    reg[GOAL] = constrain(reg[GOAL], reg[MINGOAL], reg[MAXGOAL]);
    reg[PULSE] = map(reg[GOAL], reg[MINGOAL], reg[MAXGOAL], reg[MINPULSE], reg[MAXPULSE]);

    reg[TEMP] = readTemp();

    reg[POWER] = readVcc();

    reg[VALUE4] = reg[PULSE];

    if (reg[TEMP] >= reg[MAXTEMP]) {
      reg[ENABLED] = 0;
      reg[ERRORFLAGS] |= 1<<ERRORTEMP;

    }

  }

  /*
     Things you want to do 500 times a second (@ 8mhz)
  */

  updatePos();

}
