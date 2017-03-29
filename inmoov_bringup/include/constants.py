#!/usr/bin/env python
# licensed as BSD-3

from enum import Enum

class Protocol():
    #Protocol Constants
    WRITE             = 0x03
    READ              = 0x02

    ENABLE            = 0x18
    GOALPOSITION      = 0x1E

    SERVOPIN          = 0x0A
    SENSORPIN         = 0x0B

    MINANGLE          = 0x06
    MAXANGLE          = 0x08
    MINPULSE          = 0x14
    MAXPULSE          = 0x16
    MINSENSOR         = 0xA2
    MAXSENSOR         = 0xA4
    MAXSPEED          = 0x0C
    
    GOALSPEED         = 0x20
    CALIBRATED        = 0xA0
    LED               = 0x19
    SMOOTH            = 0xA6

    PRESENTPOSITION   = 0x24
    PRESENTSPEED      = 0x26
    MOVING            = 0x2E
    SENSORRAW         = 0xA8
    POWER             = 0x2A
