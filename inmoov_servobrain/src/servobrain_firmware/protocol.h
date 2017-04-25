#ifndef PROTOCOL_H
#define PROTOCOL_H

/*
 * Protocol is bidirectional, unsigned byte is read/write + address
 * 
 * different events for read vs write
 */


enum protocolValues {
// required to be set by programmer
ID,          //00

// non-volatile writes, stored with every write
MINPULSE,    //01
MAXPULSE,    //02
MINGOAL,     //03
MAXGOAL,     //04
MINSENSOR,   //05
MAXSENSOR,   //06
MAXSPEED,    //07
PVAL,        //08
IVAL,        //09
DVAL,        //10
DEADZONE,    //11
FORCEMIN,    //12
FORCEMAX,    //13
SMOOTHING,   //14
RAMPUP,      //15
RAMPDOWN,    //16
HEARTBEAT,   //17
INDICATOR,   //18
MAXTEMP,     //19

UNUSED20,    //20
UNUSED21,    //21
UNUSED22,    //22
UNUSED23,    //23
UNUSED24,    //24
UNUSED25,    //25
UNUSED26,    //26
UNUSED27,    //27
UNUSED28,    //28
UNUSED29,    //29
UNUSED30,    //30
UNUSED31,    //31


// volatile writes, not kept between reset
GOAL,        //32
GOALSPEED,   //33
ENABLED,     //34
RESET,       //35
RAWPOSITION, //36
UNUSED36,    //37
UNUSED37,    //38
UNUSED38,    //39

// read only values, no write function
POSITION,    //40
SPEED,       //41
TEMP,        //42
POWER,       //43
CALIBRATED,  //44
ERRORFLAG,   //45
STATUSFRAME, //46
MOVING,      //47

// extended generic values
VALUE1,      //48
VALUE2,      //49
VALUE3,      //50
VALUE4,      //51
VALUE5       //52
  
};


#endif
