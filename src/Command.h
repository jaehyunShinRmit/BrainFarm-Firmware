#ifndef COMMAND_H_
#define COMMAND_H_

#include "Arduino.h"
//#include "string.h"

enum CommandCodeEnum
{
  CODE_UNDEFINED = -1,
  G00 = 0,
  G01 = 1,
  G28 = 28,
  F01 = 101,
  F02 = 102,
  F03 = 103,
  F09 = 109,
  F11 = 111,
  F12 = 112,
  F13 = 113,
  F14 = 114,
  F15 = 115,
  F16 = 116,
  F20 = 120,
  F21 = 121,
  F22 = 122,
  F31 = 131,
  F32 = 132,
  F41 = 141,
  F42 = 142,
  F43 = 143,
  F44 = 144,
  F61 = 161,
  F81 = 181,
  F82 = 182,
  F83 = 183,
  F84 = 184,
  K10 = 310,
  K11 = 311,
  K12 = 312,
  K13 = 313,
  K14 = 314,
  K15 = 315,
  K51 = 351,
  K61 = 361,
  K62 = 362,
  K63 = 363,
  K70 = 370,
  K71 = 371,
  K72 = 372,
  K73 = 373,
  K74 = 374,
  K75 = 375,
  K76 = 376,
  K77 = 377,
  K78 = 378,
  K79 = 379
};

//#define NULL 0

class Command
{
  CommandCodeEnum codeEnum;

public:
  //	Command(String);
  Command(char *commandChar);
  void print();
  CommandCodeEnum getCodeEnum();
  double getX();
  double getY();
  double getZ();
  double getS();
  long getP();
  long getV();
  long getA();
  long getB();
  long getC();
  long getW();
  long getT();
  long getE();
  long getM();
  long getQ();
  long getMD();
  unsigned int getMS(); // movingSpeed added by jaehyun Shin
  double getsysCovariance(); //added by jaehyun Shin
  double getmeasureCovariance();//added by jaehyun Shin
  long getRow();  //added by jaehyun Shin
  long getCol();  //added by jaehyun Shin


  void printQAndNewLine();

private:
  CommandCodeEnum getGCodeEnum(char *code);
  void getParameter(char *charPointer);

  double axisValue[3] = {0.0, 0.0, 0.0};
  long axisSpeedValue[3] = {0, 0, 0};
  double speedValue = 0.0;
  long parameterId = 0;
  long parameterValue = 0;
  long parameterValue2 = 0;
  long element = 0;
  long time = 0;
  long mode = 0;
  long msgQueue = 0;
 

  //add by Jaehyun Shin.
  long matrixIndex[2] = {0,0};
  double sysCovariance = 0.001;
  double measureCovariance = 0.01;
  long movingDistance = 2; // 2 second
  long movingLength   = 20; 
  unsigned int movingSpeed= 10;
};

#endif /* COMMAND_H_ */
