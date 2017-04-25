// This is an ultra simple software servo driver. For best
// results, use with a timer0 interrupt to refresh() all
// your servos once every 20 milliseconds!
// Written by Limor Fried for Adafruit Industries, BSD license

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
 
#include "SoftServo.h"
#include <TinyWireS.h>

SoftServo::SoftServo(void) {
  isAttached = false;
  servoPin = 255;
  angle = 90;
  minPulse=550;
  maxPulse=2500;
}

void SoftServo::attach(uint8_t pin) {
  servoPin = pin;
  angle = 90;
  isAttached = true;
  pinMode(servoPin, OUTPUT);
}

void SoftServo::detach(void) {
  isAttached = false;
  pinMode(servoPin, INPUT);
}

boolean  SoftServo::attached(void) {
  return isAttached;
}

void SoftServo::write(uint8_t a) {
  angle = a;

  if (! isAttached) return;
  pulse = map(a, 0, 180, minPulse, maxPulse);  
}

void SoftServo::writeMicroseconds(uint16_t a) {
  a = constrain(a,minPulse,maxPulse);
  angle = map(a,minPulse,maxPulse,0,180);

  if (! isAttached) return;
  pulse = a;  
}

void SoftServo::refresh(void) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servoPin, LOW);
}
/*
// Implement a delay loop that checks for the stop bit (basically direct copy of the stock arduino implementation from wiring.c)
void SoftServo::tws_delayMicroseconds(unsigned long us)
{
  
  micros();
    uint16_t start = (uint16_t)micros();
    while (us > 0)
    {
        TinyWireS_stop_check();
        if (((uint16_t)micros() - start) >= 1)
        {
            us--;
            start += 1;
        }
    }
    
}
*/
// Implement a delay loop that checks for the stop bit (basically direct copy of the stock arduino implementation from wiring.c)
void SoftServo::tws_delayMicroseconds(unsigned long us)
{
  
  micros();
    uint16_t start = (uint16_t)micros();
    while ((micros() - start) <= us)
    {
        TinyWireS_stop_check();

    }
    
}

