#include <EEPROM.h>
#include <Servo.h>
#include "TeensyServo.h"
#include "configuration.h"
#include <ros.h>
#include <inmoov_msgs/MotorCommand.h>
#include <inmoov_msgs/MotorParameter.h>
#include <inmoov_msgs/MotorStatus.h>
#include <std_srvs/Empty.h>

#define LED 13

int i, j;
int updateMillis;
int commands;

TeensyServo *tServo[12];

int commandAngle;
int startMillis;

ros::NodeHandle  nh;

inmoov_msgs::MotorCommand command_msg;
inmoov_msgs::MotorParameter parameter_msg;
inmoov_msgs::MotorStatus status_msg;

const bool heartbeats[] = {1, 0, 1, 0, 0, 0, 0, 0};

void getParameter(const inmoov_msgs::MotorParameter::Request & req, inmoov_msgs::MotorParameter::Response & res) {
  byte id = req.id;
  byte parameter = req.parameter;
  float value = 0.0;

  switch (parameter) {

    case P_GOALPOSITION:
      value = tServo[id]->getGoal();
      break;

    case P_MINANGLE:
      value = tServo[id]->getMinAngle();
      break;

    case P_MAXANGLE:
      value = tServo[id]->getMaxAngle();
      break;

    case P_MINPULSE:
      value = (float)tServo[id]->getMinPulse();
      break;

    case P_MAXPULSE:
      value = (float)tServo[id]->getMaxPulse();
      break;

    case P_MINSENSOR:
      value = (float)tServo[id]->getMinSensor();
      break;

    case P_MAXSENSOR:
      value = (float)tServo[id]->getMaxSensor();
      break;

    case P_CALIBRATED:
      value = tServo[id]->getCalibrated();
      break;

    case P_PRESENTPOSITION:
      value = tServo[id]->readPositionAngle();
      break;

    case P_SENSORRAW:
      value = (float)tServo[id]->readPositionRaw();
      break;

    case P_MOVING:
      value = (float)tServo[id]->getMoving();
      break;

    case P_PRESENTSPEED:
      value = tServo[id]->readPresentSpeed();
      break;

    case P_SMOOTH:
      value = (float)tServo[id]->getSmooth();
      break;

    case P_GOALSPEED:
      value = tServo[id]->getMaxSpeed();
      break;

    case P_ENABLE:
      value = (float)tServo[id]->getEnabled();
      break;

    case P_POWER:
      value = (float)tServo[id]->getPower();
      break;
  }

  res.data = value;

}

void commandCb( const inmoov_msgs::MotorCommand& command_msg) {

  byte id = command_msg.id;
  byte parameter = command_msg.parameter;
  float value = command_msg.value;

  switch (parameter) {

    case P_GOALPOSITION:
      tServo[id]->setGoal(value);
      //String string = "Goal Position = " + String(tServo[id]->commandPulse);
      //nh.loginfo(String(tServo[id]->commandPulse));
      break;

    case P_MINANGLE:
      tServo[id]->setMinAngle(value);
      break;

    case P_MAXANGLE:
      tServo[id]->setMaxAngle(value);
      break;

    case P_MINPULSE:
      tServo[id]->setMinPulse(value);
      break;

    case P_MAXPULSE:
      tServo[id]->setMaxPulse(value);
      break;

    case P_ENABLE:
      tServo[id]->setEnabled(value);
      break;

    case P_CALIBRATED:
      tServo[id]->setCalibrated(value);
      break;

    case P_MINSENSOR:
      tServo[id]->setMinSensor(value);
      break;

    case P_MAXSENSOR:
      tServo[id]->setMaxSensor(value);
      break;

    case P_SMOOTH:
      tServo[id]->setSmooth(value);
      break;

    case P_GOALSPEED:
      tServo[id]->setMaxSpeed(value);

  }


}


ros::Publisher motorstatus("motorstatus", &status_msg);

ros::Subscriber<inmoov_msgs::MotorCommand> motorcommand("motorcommand", &commandCb);

ros::ServiceServer<inmoov_msgs::MotorParameter::Request, inmoov_msgs::MotorParameter::Response> server("motorparameter", &getParameter);


void setupADC() {
  analogReadResolution(12);
  analogReference(EXTERNAL);
}


void updateServos() {
  for (i = 0; i < NUMSERVOS; i++) {
    tServo[i]->update();
  }
}

void setupServos() {
  tServo[0] = new TeensyServo(0, A8);
  tServo[1] = new TeensyServo(1, A9);
  tServo[2] = new TeensyServo(2, A10);
  tServo[3] = new TeensyServo(3, A11);
  tServo[4] = new TeensyServo(4, A7);
  tServo[5] = new TeensyServo(5, A6);
  tServo[6] = new TeensyServo(6, A5);
  tServo[7] = new TeensyServo(7, A4);
  tServo[8] = new TeensyServo(8, A3);
  tServo[9] = new TeensyServo(9, A2);
  tServo[10] = new TeensyServo(10, A1);
  tServo[11] = new TeensyServo(11, A0);

}

byte generateChecksum() {
  return 0;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);

  nh.initNode();
  nh.advertise(motorstatus);
  nh.subscribe(motorcommand);
  nh.advertiseService(server);

  while (!nh.connected() ) {
    nh.spinOnce();
  }

  setupADC();
  delay(1);

  setupServos();

  Serial.begin(115200);

  startMillis = millis();
  updateMillis = millis();
  commands = 0;



  nh.loginfo("Setup Complete!!!");

}

int pushmotorstatus = 0;  // which motorstatus to update

void loop() {

  updateServos();

  digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);
  /*
    if ((millis() - updateMillis) >= (UPDATEPERIOD / NUMSERVOS)) {
        status_msg.id           = pushmotorstatus;
        status_msg.goal         = tServo[pushmotorstatus]->getGoal();
        status_msg.position     = tServo[pushmotorstatus]->readPositionAngle();
        status_msg.presentspeed = tServo[pushmotorstatus]->readPresentSpeed();
        status_msg.moving       = tServo[pushmotorstatus]->getMoving();
        //status_msg.posraw       = tServo[servo]->sampleDuration;
        status_msg.posraw       = tServo[pushmotorstatus]->readPositionRaw();
        status_msg.enabled      = tServo[pushmotorstatus]->getEnabled();
        status_msg.power        = tServo[pushmotorstatus]->getPower();

        motorstatus.publish( &status_msg);

        pushmotorstatus++;
        if(pushmotorstatus == NUMSERVOS) {
          pushmotorstatus = 0;
        }

        updateMillis = millis();
        nh.spinOnce();


      }
  */


  if ((millis() - updateMillis) >= UPDATEPERIOD) {
    for (int servo = 0; servo < NUMSERVOS; servo++) {
      
      status_msg.id           = servo;
      status_msg.goal         = tServo[servo]->getGoal();
      status_msg.position     = tServo[servo]->readPositionAngle();
      status_msg.presentspeed = tServo[servo]->readPresentSpeed();
      status_msg.moving       = tServo[servo]->getMoving();
      //status_msg.posraw       = tServo[servo]->sampleDuration;
      status_msg.posraw       = tServo[servo]->readPositionRaw();
      status_msg.enabled      = tServo[servo]->getEnabled();
      status_msg.power        = tServo[servo]->getPower();

      nh.spinOnce();

      motorstatus.publish( &status_msg);

      nh.spinOnce();
    }

    updateMillis = millis();
    nh.spinOnce();

    commands = 0;

  }

  nh.spinOnce();

}



