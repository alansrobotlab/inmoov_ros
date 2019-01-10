#ifndef configuration_h
#define configuration_h

#define NUMSERVOS 12

#define MINPULSE 550
#define MAXPULSE 2450

#define MINSENSOR 550
#define MAXSENSOR 3800

#define MINANGLE 0
#define MAXANGLE 180

#define SMOOTH 0
#define MAXSPEED 100

#define ID_ADDRESS 512

#define EEPROM_RECORD_SIZE 64

#define EEPROM_START 0

#define UPDATEPERIOD 250

//Protocol Constants
#define P_WRITE  0x03
#define P_READ 0x02

#define P_GOALPOSITION 0x1E
#define P_ENABLE 0x18
#define P_MINANGLE 0x06
#define P_MAXANGLE 0x08
#define P_MINPULSE 0x14
#define P_MAXPULSE 0x16
#define P_MINSENSOR 0xA2
#define P_MAXSENSOR 0xA4
#define P_GOALSPEED 0x20
#define P_CALIBRATED 0xA0
#define P_LED 0x19
#define P_SMOOTH 0xA6

#define P_PRESENTPOSITION 0x24
#define P_PRESENTSPEED 0x26
#define P_MOVING 0x2E
#define P_SENSORRAW 0xA8
#define P_POWER 0x2A


#endif
