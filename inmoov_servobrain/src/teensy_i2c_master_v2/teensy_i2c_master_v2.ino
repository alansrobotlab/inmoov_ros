// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#define ID          00
#define MINPULSE    01
#define MAXPULSE    02
#define MINGOAL     03
#define MAXGOAL     04
#define SENSMIN     05
#define SENSMAX     06
#define MAXSPEED    07
#define PVAL        08
#define IVAL        09
#define DVAL        10
#define DEADZONE    11
#define FORCEMIN    12
#define FORCEMAX    13

#define SAMPLE_DUR  22
#define SAMPLES     23
#define RAWPOSITION 24
#define TEMP        25
#define GOAL        26
#define POSITION    27
#define ENABLED     28
#define POWER       29
#define CALIBRATED  30
#define SPEED       31

#define I2C_DELAY   0  //per packet delay seems to work at 100kbps
//#define I2C_DELAY   100  //per packet delay seems to work at 200kbps

#include <i2c_t3.h>
//#include <Wire.h>


#define LED 13

const bool heartbeats[] = { 1, 0, 1, 0, 0, 0, 0, 0 };

int values[100];
int sortedvalues[100];
byte sortedcount[100];
float average;
float minimum;
float maximum;

union CShort {
  byte b[2];
  signed short val;
} cshort;

void setup()
{
  Serial.begin(9600);

  pinMode(LED, OUTPUT); //LED on Model A

  Wire.begin(); // join i2c bus (address optional for master)
  // 511us per message at 100kbps
  // 287us per message at 200kbps
  // 215us per message at 300kbps
  // 181us per message at 400kbps
  Wire.setRate(I2C_RATE_200);
  Wire.setDefaultTimeout(5000);
}

byte x = 0;

long int start = 0;

void loop() {

  for (int a = 450; a < 1350; a++) {

    digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);

    writeServoRegister(8, GOAL, a);

    readServoRegister(8, POSITION);
    //Serial.println(readServoRegister(8, POSITION));

    delay(5);
  }

  for (int a = 1350; a > 450; a--) {

    digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);

    writeServoRegister(8, GOAL, a);

    readServoRegister(8, POSITION);
    //Serial.println(readServoRegister(8, POSITION));

    delay(1);
  }
  //Serial.println(readServoRegister(8, SAMPLE_DUR));

  writeServoRegister(8, GOAL, 1800);

  start = millis();
  for (int i = 0; i < 1000; i++) {
    //readServoRegister(8, POSITION);
    //for (int j = 0; j < 30; j++) 
    {
      writeServoRegister(8, POWER, 512);
      readServoRegister(8, POSITION);
    }
  }
  Serial.print("Duration for 2000 messages, 1000 writes/reads to servo:  ");
  Serial.print(millis() - start);
  Serial.println(" milliseconds");
  Serial.print("Samples Span:  ");
  Serial.println(readServoRegister(8, SAMPLE_DUR));
  Serial.print("messages recieved:  ");
  Serial.println(readServoRegister(8, POWER));
}

short readServoRegister(byte servo, byte reg) {
  Wire.beginTransmission(servo); // transmit to device #8
  Wire.write(reg);        // sends five bytes
  Wire.endTransmission(true);    // stop transmitting
  Wire.requestFrom( servo, 2, true);
  //while (Wire.available() < 4) {
  //  cfloat.fval = -1.0f;
  //}
  cshort.val = 666;
  for (int i = 0; i < 2; i++) {
    cshort.b[i] = Wire.read();
  }
  //delayMicroseconds(I2C_DELAY);
  return cshort.val;
}

void writeServoRegister(byte servo, byte reg, short value) {
  Wire.beginTransmission(servo); // transmit to device #8
  Wire.write(reg);        // sends five bytes
  cshort.val = value;
  Wire.write(cshort.b, 2);
  Wire.endTransmission();    // stop
  //delayMicroseconds(I2C_DELAY);
}




