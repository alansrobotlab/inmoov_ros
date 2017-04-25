#!/usr/bin/env python
# licensed as BSD-3

class Servo:
    key = -1
    name = ""
    bus = -1

    goal = -1.0

    servoPin = -1

    minPulse = -1.0
    maxPulse = -1.0
    minGoal = -1.0
    maxGoal = -1.0
    rest = -1.0
    smoothing = -1
    maxSpeed = -1.0

    sensorPin = -1
    minSensor = -1.0
    maxSensor = -1.0