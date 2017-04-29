#include <Servo.h>
#include "TeensyServo.h"
#include "configuration.h"
#include <ros.h>
#include <inmoov_msgs/MotorCommand.h>
#include <inmoov_msgs/MotorParameter.h>
#include <inmoov_msgs/SmartServoStatus.h>
#include <std_srvs/Empty.h>
#include <i2c_t3.h>
#include "protocol.h"

#define I2C_DELAY   0  //per packet delay seems to work at 100kbps
//#define I2C_DELAY   100  //per packet delay seems to work at 200kbps


#define LED 13

int i, j;
int updateMillis;
int commands;

TeensyServo *tServo[NUMSERVOS];

int commandAngle;
int startMillis;

ros::NodeHandle  nh;

inmoov_msgs::MotorCommand command_msg;
inmoov_msgs::MotorParameter parameter_msg;
inmoov_msgs::SmartServoStatus status_msg;

const bool heartbeats[] = {1, 0, 1, 0, 0, 0, 0, 0};

union CShort {
  byte b[2];
  signed short val;
} cshort;

void getParameter(const inmoov_msgs::MotorParameter::Request & req, inmoov_msgs::MotorParameter::Response & res) {
  byte id = req.id;
  byte parameter = req.parameter;
  float value = 0.0;

  switch (parameter) {

    case P_GOALPOSITION:
      value = readServoRegister(id, GOAL);
      value /= 100.0;
      //value = tServo[id]->getGoal();
      break;

    case P_MINANGLE:
      value = readServoRegister(id, MINGOAL);
      value /= 100.0;
      //value = tServo[id]->getMinAngle();
      break;

    case P_MAXANGLE:
      value = readServoRegister(id, MAXGOAL);
      value /= 100.0;
      //value = tServo[id]->getMaxAngle();
      break;

    case P_MINPULSE:
      value = readServoRegister(id, MINPULSE);
      //value = (float)tServo[id]->getMinPulse();
      break;

    case P_MAXPULSE:
      value = readServoRegister(id, MAXPULSE);
      //value = (float)tServo[id]->getMaxPulse();
      break;

    case P_MINSENSOR:
      value = readServoRegister(id, MINSENSOR);
      //value = (float)tServo[id]->getMinSensor();
      break;

    case P_MAXSENSOR:
      value = readServoRegister(id, MAXSENSOR);
      //value = (float)tServo[id]->getMaxSensor();
      break;

    case P_CALIBRATED:
      value = readServoRegister(id, CALIBRATED);
      //value = tServo[id]->getCalibrated();
      break;

    case P_PRESENTPOSITION:
      value = readServoRegister(id, POSITION);
      value /= 100.0;
      //value = tServo[id]->readPositionAngle();
      break;

    case P_SENSORRAW:
      value = readServoRegister(id, RAWPOSITION);
      //value = (float)tServo[id]->readPositionRaw();
      break;

    case P_MOVING:
      value = readServoRegister(id, MOVING);
      //value = (float)tServo[id]->getMoving();
      break;

    case P_PRESENTSPEED:
      value = readServoRegister(id, SPEED);
      value /= 100.0;
      //value = tServo[id]->readPresentSpeed();
      break;

    case P_SMOOTH:
      value = readServoRegister(id, SMOOTHING);
      //value = (float)tServo[id]->getSmooth();
      break;

    case P_GOALSPEED:
      value = readServoRegister(id, GOALSPEED);
      value /= 100.0;
      //value = tServo[id]->getMaxSpeed();
      break;

    case P_ENABLE:
      value = readServoRegister(id, ENABLED);
      //value = (float)tServo[id]->getEnabled();
      break;

    case P_POWER:
      value = readServoRegister(id, POWER);
      //value = (float)tServo[id]->getPower();
      break;
  }

  res.data = value;

}

void commandCb( const inmoov_msgs::MotorCommand& command_msg) {

  byte id = command_msg.id;
  byte parameter = command_msg.parameter;
  float value = command_msg.value;

  signed short shortVal = short(value);

  switch (parameter) {

    case P_GOALPOSITION:
      shortVal = short(value * 100.0);
      writeServoRegister(id, GOAL, shortVal);
      //tServo[id]->setGoal(value);

      //String string = "Goal Position = " + String(tServo[id]->commandPulse);
      //nh.loginfo(String(tServo[id]->commandPulse));
      break;

    case P_MINANGLE:
      shortVal = short(value * 100.0);
      writeServoRegister(id, MINGOAL, shortVal);
      //tServo[id]->setMinAngle(value);
      break;

    case P_MAXANGLE:
      shortVal = short(value * 100.0);
      writeServoRegister(id, MAXGOAL, shortVal);
      //tServo[id]->setMaxAngle(value);
      break;

    case P_MINPULSE:
      writeServoRegister(id, MINPULSE, shortVal);
      //tServo[id]->setMinPulse(value);
      break;

    case P_MAXPULSE:
      writeServoRegister(id, MAXPULSE, shortVal);
      //tServo[id]->setMaxPulse(value);
      break;

    case P_ENABLE:
      writeServoRegister(id, ENABLED, shortVal);
      //tServo[id]->setEnabled(value);
      break;

    case P_CALIBRATED:
      writeServoRegister(id, CALIBRATED, shortVal);
      //tServo[id]->setCalibrated(value);
      break;

    case P_MINSENSOR:
      writeServoRegister(id, MINSENSOR, shortVal);
      //tServo[id]->setMinSensor(value);
      break;

    case P_MAXSENSOR:
      writeServoRegister(id, MAXSENSOR, shortVal);
      //tServo[id]->setMaxSensor(value);
      break;

    case P_SMOOTH:
      writeServoRegister(id, SMOOTHING, shortVal);
      //tServo[id]->setSmooth(value);
      break;

    case P_GOALSPEED:
      shortVal = short(value * 100.0);
      writeServoRegister(id, GOALSPEED, shortVal);
      //tServo[id]->setMaxSpeed(value);

  }

  //generateMotorStatus();
  
}


ros::Publisher smartservostatus("smartservostatus", &status_msg);

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
  tServo[0] = new TeensyServo(8);


}

byte generateChecksum() {
  return 0;
}

void setup() {

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.begin(); // join i2c bus (address optional for master)
  // 511us per message at 100kbps
  // 287us per message at 200kbps
  // 215us per message at 300kbps
  // 181us per message at 400kbps
  //Wire.setRate(I2C_RATE_200);
  Wire.setDefaultTimeout(20000);

  nh.initNode();
  nh.advertise(smartservostatus);
  nh.subscribe(motorcommand);
  nh.advertiseService(server);

  while (!nh.connected() ) {
    nh.spinOnce();
  }

  setupADC();
  delay(1);

  setupServos();

  //Serial.begin(115200);

  startMillis = millis();
  updateMillis = millis();
  commands = 0;



  nh.loginfo("Setup Complete!!!");

}

int pushmotorstatus = 0;  // which motorstatus to update

char joint[2] = " ";
byte bus = 0;

void loop() {

  updateServos();

  digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);

  if ((millis() - updateMillis) >= UPDATEPERIOD) {
    generateMotorStatus();

  }

  nh.spinOnce();

}

void generateMotorStatus() {
  for (int servo = 0; servo < NUMSERVOS; servo++) {


    status_msg.joint        = joint;
    status_msg.bus          = bus;
    status_msg.id           = readServoRegister(8, ID);
    status_msg.goal         = readServoRegister(8, GOAL) / 100.0;
    status_msg.position     = readServoRegister(8, POSITION) / 100.0;
    status_msg.presentspeed = readServoRegister(8, SPEED) / 100.0;
    status_msg.moving       = readServoRegister(8, MOVING);
    status_msg.posraw       = readServoRegister(8, RAWPOSITION);
    status_msg.enabled      = readServoRegister(8, ENABLED);
    status_msg.power        = readServoRegister(8, POWER);
    status_msg.temp         = readServoRegister(8, TEMP);
    status_msg.error        = readServoRegister(8, ERRORFLAG);
    status_msg.value1       = readServoRegister(8, VALUE1);
    status_msg.value2       = readServoRegister(8, VALUE2);
    status_msg.value3       = readServoRegister(8, VALUE3);
    status_msg.value4       = readServoRegister(8, VALUE4);
    status_msg.value5       = readServoRegister(8, VALUE5);

    //writeServoRegister(8, PVAL ,128);



    nh.spinOnce();

    smartservostatus.publish( &status_msg);

    nh.spinOnce();
  }

  updateMillis = millis();
  nh.spinOnce();

  commands = 0;
}

short readServoRegister(byte servo, byte reg) {
  byte checksum = ~(reg );
  Wire.beginTransmission(servo); // transmit to device #8
  Wire.write(reg);  
  //Wire.write(READREGISTER);      
  Wire.write(checksum);
  Wire.endTransmission(I2C_NOSTOP);    // stop transmitting
  Wire.requestFrom( servo, 4, I2C_STOP);

  byte responseRegister = Wire.read();

  cshort.val = -1;
  for (int i = 0; i < 2; i++) {
    cshort.b[i] = Wire.read();
  }

  byte responseChecksum = Wire.read();
  //delayMicroseconds(I2C_DELAY);

  byte calculatedChecksum = ~(responseRegister + cshort.b[0] + cshort.b[1]);

  if ((reg == responseRegister) & (calculatedChecksum == responseChecksum)) {
    return cshort.val;
  }
  else {
    return -1;
  }
  
  
}

void writeServoRegister(byte servo, byte reg, short value) {
  cshort.val = value;
  byte checksum = ~(reg  + cshort.b[0] + cshort.b[1]);
  Wire.beginTransmission(servo); // transmit to device #8
  Wire.write(reg);        // sends five bytes
  //Wire.write(WRITEREGISTER);
  Wire.write(cshort.b[0]);
  Wire.write(cshort.b[1]);
  Wire.write(checksum);
  Wire.endTransmission();    // stop

  //nh.loginfo(String(readServoRegister(8, GOAL), DEC).toCharArray());
  //delayMicroseconds(I2C_DELAY);
}

