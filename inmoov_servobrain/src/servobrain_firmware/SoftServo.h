// This is an ultra simple software servo driver. For best
// results, use with a timer0 interrupt to refresh() all
// your servos once every 20 milliseconds!
// Written by Limor Fried for Adafruit Industries, BSD license

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
 
//class Adafruit_SoftServo {
class SoftServo {
 public:
  SoftServo(void);
  void attach(uint8_t pin);
  void detach();
  boolean attached();
  void write(uint8_t a);
  void writeMicroseconds(uint16_t a);
  void refresh(void);
  void setMinimumPulse(uint16_t);  // pulse length for 0 degrees in microseconds, 540uS default
  void setMaximumPulse(uint16_t);  // pulse length for 180 degrees in microseconds, 2400uS default

 private:
  boolean isAttached;
  uint8_t servoPin, angle;
  uint16_t pulse;
  uint16_t  minPulse;      // minimum pulse, 16uS units  (default is 34)
  uint16_t  maxPulse;      // maximum pulse, 16uS units, 0-4ms range (default is 150)
  void tws_delayMicroseconds(unsigned long us);

};

