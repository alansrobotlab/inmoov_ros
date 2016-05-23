#include <Servo.h>
#include <WProgram.h>
#include "eeprom.h"
#include "configuration.h"
#include <ros.h>

class TeensyServo {
  private:
    Eeprom e;

    void readEeprom(int);
    void writeEeprom();
    int generateEepromChecksum();
    
    float goalAngle;

  public:
    int servoPin;
    int sensorPin;

    int debugInt = 0;
    float debugFloat = 0;


    bool moving = false;
    bool power = false;
    int r;
    int deltaPulse;
    int velocityArc;
    int moveDuration;
    int enabled = 0;

    int startMillis;
    int startPulse;
    int commandPulse;
    int currentPulse;

    int deltaMillis;
    int ticksPerSecond = 1500;

    int i, j;

    Servo servo;

    TeensyServo(int, int);

    void setGoal(float);
    float getGoal();

    void moveToMicroseconds(int);
    short readPositionRaw();
    float readPositionAngle();
    short readPositionPulse();

    void update();

    void calibrate();

    void setupADC();

    void setMinPulse(short);
    short getMinPulse();
    void setMaxPulse(short);
    short getMaxPulse();

    void setMinAngle(float);
    float getMinAngle();
    void setMaxAngle(float);
    float getMaxAngle();

    void setMinSensor(int);
    int getMinSensor();
    void setMaxSensor(int);
    int getMaxSensor();

    void setEnabled(bool);
    bool getEnabled();

    bool getCalibrated();
    void setCalibrated(bool);


    void setMaxSpeed(float);
    float getMaxSpeed();

    void setSmooth(byte);
    byte getSmooth();

    bool getPower();

    bool getMoving();

    float readPresentSpeed();

};
