#include <Servo.h>
#include <WProgram.h>
#include "configuration.h"
#include "TeensyServo.h"
#include <ros.h>
#include "protocol.h"


TeensyServo::TeensyServo(int pin) {
  this->servoPin = pin;
  
  pinMode(servoPin, OUTPUT);
  //pinMode(sensorPin, INPUT);

  //readEeprom(servoPin);

  //setGoal(readPositionAngle());

  moving = false;
  enabled = 0;

  sampleStartMillis = millis();

}


void TeensyServo::setGoal(float a) {
  /*
    goalAngle = a;
    if (a > e.maxAngle)
    goalAngle = e.maxAngle;
    if (a < e.minAngle)
    goalAngle = e.minAngle;
  */

  receivedCommand = true;   //mark that we've received a position command

  // no guarantees the min max angles are mapped backwards
  goalAngle = a;
  if (e.minAngle < e.maxAngle) {
    goalAngle = constrain(goalAngle, e.minAngle, e.maxAngle);
  }
  else {
    goalAngle = constrain(goalAngle, e.maxAngle, e.minAngle);
  }


  //this->moveToMicroseconds((map(goalAngle, (int)(e.minAngle * 1000), (int)(e.maxAngle * 1000.0), e.minPulse, e.maxPulse) / 1000.0));

  this->moveToMicroseconds((map(goalAngle * 1000, (int)(e.minAngle * 1000.0), (int)(e.maxAngle * 1000.0), e.minPulse, e.maxPulse)));

  //nh.loginfo("SetGoal!!!");
}


float TeensyServo::getGoal() {
  return goalAngle;
}


void TeensyServo::moveToMicroseconds(int microseconds) {
  this->startMillis = millis();
  this->startPulse = this->readPositionPulse();
  //this->startPulse = commandPulse;



  if(e.minPulse < e.maxPulse) {
    microseconds = constrain(microseconds,e.minPulse,e.maxPulse);
  }
  else {
    microseconds = constrain(microseconds,e.maxPulse,e.minPulse);
  }

  /*
  if (microseconds > e.maxPulse)
    microseconds = e.maxPulse;
  if (microseconds < e.minPulse)
    microseconds = e.minPulse;
  */
  commandPulse = microseconds;

  deltaPulse = ((commandPulse - startPulse)); //*1000)/ticksPerSecond;
  moveDuration = abs((deltaPulse * 1000) / ticksPerSecond);

  if (deltaPulse > 0) {
    velocityArc = 0;
  }
  else {
    velocityArc = 180;
  }

  moving = true;
}



short TeensyServo::readPositionRaw() {
  /*
    long int retval =0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
    retval += analogRead(this->sensorPin);
    }
    retval /= NUM_SAMPLES;

    return retval;
  */
  return this->position;

}


#define NUM_SAMPLES 350
void TeensyServo::updatePosition() {
  for (int i = 0; i < 50; i++) {
    sampleBucket += analogRead(this->sensorPin);
  }
  sampleCount += 50;

  if (sampleCount == NUM_SAMPLES) {
    sampleCount = 0;
    this->position = sampleBucket /= NUM_SAMPLES;
    this->sampleDuration = millis() - this->sampleStartMillis;
    this->sampleStartMillis = millis();
  }
}

/*
  #define NUM_READS 16
  short TeensyServo::readPositionRaw_old() {
  // http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/
  // return analogRead(this->sensorPin);
  // read multiple values and sort them to take the mode
  int sortedValues[NUM_READS];
  for (int i = 0; i < NUM_READS; i++) {
    int value = analogRead(this->sensorPin);
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (int k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  returnval = (int)returnval / 10;

  return returnval * 1100 / 1023;


  }
*/


float TeensyServo::readPositionAngle() {
  short p = readPositionRaw();

  if (p < 100)
    return 0;
  else
    return ((map(p, e.minSensor, e.maxSensor, (int)(e.minAngle * 1000.0), (int)(e.maxAngle * 1000.0))) / 1000.0);
}

short TeensyServo::readPositionPulse() {
  return map(this->readPositionRaw(), e.minSensor, e.maxSensor, e.minPulse, e.maxPulse);
}

void TeensyServo::update() {

  ////Serial.println("Servo Update!");

  this->updatePosition();


  deltaMillis = millis() - startMillis;

  switch (e.smooth) {

    case 0:  // direct
    case 1:  // maxspeed (placeholder)
      this->servo.writeMicroseconds(commandPulse);
      break;

    case 2: // linear
      if (commandPulse > startPulse) {
        currentPulse = startPulse + (ticksPerSecond * this->deltaMillis) / 1000;

        if (currentPulse > commandPulse)
          currentPulse = commandPulse;
      }
      if (commandPulse < startPulse) {
        currentPulse = startPulse - (ticksPerSecond * this->deltaMillis) / 1000;

        if (this->currentPulse < this->commandPulse)
          this->currentPulse = this->commandPulse;
      }
      servo.writeMicroseconds(currentPulse);
      break;

    case 3: // calcCos1
      if (deltaPulse > 0) {
        velocityArc = 0 + map(millis() - startMillis, 0, moveDuration, 0, 180);
        if (velocityArc >= 180) {
          velocityArc = 180;
          moving = false;
        }

        r = (cos(radians(velocityArc)) * 100);
        currentPulse = map(r, 100, -100, startPulse, commandPulse);
      }
      if (deltaPulse < 0) {
        velocityArc = 180 - map(millis() - startMillis, 0, moveDuration, 0, 180);
        if (velocityArc <= 0) {
          velocityArc = 0;
          moving = false;
        }
        r = (cos(radians(velocityArc)) * 100);
        currentPulse = map(r, -100, 100, startPulse, commandPulse);
      }
      if (deltaPulse == 0) {
        moving = false;
      }
      servo.writeMicroseconds(currentPulse);
      break;

    case 4:
      if (deltaPulse > 0) {
        velocityArc = 0 + map(millis() - startMillis, 0, moveDuration, 0, 180);
        if (velocityArc >= 180) {
          velocityArc = 180;
          moving = false;
        }
        if (velocityArc <= 90) {
          r = sqrt(cos(radians(velocityArc))) * 100;
          currentPulse = map(r, 100, -100, startPulse, commandPulse);
        }
        else {
          r = -sqrt(cos(radians(180 - velocityArc))) * 100;
          currentPulse = map(r, 100, -100, startPulse, commandPulse);
        }
      }
      if (deltaPulse < 0) {
        velocityArc = 180 - map(millis() - startMillis, 0, moveDuration, 0, 180);
        if (velocityArc <= 0) {
          velocityArc = 0;
          moving = false;
        }
        if (velocityArc <= 90) {
          r = sqrt(cos(radians(velocityArc))) * 100;
          currentPulse = map(r, -100, 100, startPulse, commandPulse);
        }
        else {
          r = -sqrt(cos(radians(180 - velocityArc))) * 100;
          currentPulse = map(r, -100, 100, startPulse, commandPulse);
        }
      }
      if (deltaPulse == 0) {
        moving = false;
      }
      servo.writeMicroseconds(currentPulse);
      break;
      moving = false;
  }

}

/*
  void TeensyServo::calibrate() {
  smooth=2;
  this->setGoal(e.minAngle);
  while(moving==true) {
    this->update();
    delay(1);
  }
  delay(1000);
  e.minSensor=readPositionRaw();

    this->setGoal(e.maxAngle);
  while(moving==true) {
    this->update();
    delay(1);
  }
  delay(1000);
  e.maxSensor=readPositionRaw();
  e.calibrated=true;
  smooth=0;
  }
*/

void TeensyServo::setupADC() {
  analogReadResolution(12);
  analogReference(EXTERNAL);
  //analogReadAveraging(8);
}

void TeensyServo::setMinPulse(short minpulse) {
  e.minPulse = minpulse;
  ////writeEeprom();
  setEnabled(0);
  //setEnabled(1);
  e.calibrated = false;
  moving = false;
}

short TeensyServo::getMinPulse() {
  return e.minPulse;

}

void TeensyServo::setMaxPulse(short maxpulse) {
  e.maxPulse = maxpulse;
  //writeEeprom();
  setEnabled(0);
  //setEnabled(1);
  e.calibrated = false;
  moving = false;
}

short TeensyServo::getMaxPulse() {
  return e.maxPulse;

}

void TeensyServo::setMinAngle(float minangle) {
  e.minAngle = minangle;
  e.calibrated = false;
  //writeEeprom();
  moving = false;
}

float TeensyServo::getMinAngle() {
  return e.minAngle;
}

void TeensyServo::setMaxAngle(float maxAngle) {
  e.maxAngle = maxAngle;
  e.calibrated = false;
  //writeEeprom();
  moving = false;
}

float TeensyServo::getMaxAngle() {
  //return 180;
  return e.maxAngle;
}

void TeensyServo::setMinSensor(int minsensor) {
  e.minSensor = minsensor;
  e.calibrated = false;
  //writeEeprom();
  moving = false;
}

int TeensyServo::getMinSensor() {
  return e.minSensor;
}

void TeensyServo::setMaxSensor(int maxsensor) {
  e.maxSensor = maxsensor;
  e.calibrated = false;
  //writeEeprom();
  moving = false;
}

int TeensyServo::getMaxSensor() {
  return e.maxSensor;
}

void TeensyServo::setEnabled(bool val) {

  if (val != 0) {
    // if we haven't, just readposition and use that instead.
    if (receivedCommand == false) {
      setGoal(readPositionAngle());
    }
    
    servo.attach(this->servoPin, e.minPulse, e.maxPulse); //,readPositionPulse());
    enabled = 1;
  }
  else {
    servo.detach();
    enabled = 0;
  }
}

bool TeensyServo::getEnabled() {
  return enabled;
}


bool TeensyServo::getCalibrated() {
  return e.calibrated;
}

void TeensyServo::setCalibrated(bool c) {
  e.calibrated = c;
  //writeEeprom();
}






int TeensyServo::generateEepromChecksum() {

  return 14;
}

void TeensyServo::setSmooth(byte s) {
  e.smooth = s;
  //writeEeprom();
}

byte TeensyServo::getSmooth() {
  return e.smooth;
}

float TeensyServo::getMaxSpeed() {
  return e.maxSpeed;
}

void TeensyServo::setMaxSpeed(float s) {
  e.maxSpeed = s;
  //writeEeprom();
}

float TeensyServo::readPresentSpeed() {
  int speed;
  speed = goalAngle - readPositionAngle();
  if (speed > 0) {
    speed = 1;
  }
  if (speed < 0) {
    speed = -1;
  }
  return speed;
}

bool TeensyServo::getMoving() {

  if (abs(readPositionAngle() - goalAngle) < 2)
    return 0;
  else
    return 1;

}

bool TeensyServo::getPower() {
  if (readPositionRaw() > 100)
    return 1;
  else
    return 0;
}


