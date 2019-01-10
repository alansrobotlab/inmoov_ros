#ifndef eeprom_h
#define eeprom_h

struct Eeprom {

  byte id;

  byte checksum;

  float minAngle;
  float maxAngle;

  short minPulse; //defined as pulse for 0 degrees
  short maxPulse; //defined as pulse for 180 degrees

  short minSensor; //defined as sensor reading for minangle
  short maxSensor; //defined as sensor reading for maxangle

  short maxSpeed; //rpm?

  byte smooth; // smothing algorythm setting
  // 0 == instant
  // 1 == max speed
  // 2 == linear
  // 3 == cos
  // 4 == ??? (I forgot)

  byte calibrated;  // 1 is calibrated, 0 is not calibrated

  float p;    // PID P value * 100
  float i;    // PID I value * 100
  float d;    // PID D value * 100
};


#endif
