/*
Modified to add command functions for Frambot-sensor
sensorCode = 'K'
*/

#include "Command.h"

const char axisCodes[3] = {'X', 'Y', 'Z'};
const char axisSpeedCodes[3] = {'A', 'B', 'C'};
const char speedCode = 'S';
const char parameterIdCode = 'P';
const char parameterValueCode = 'V';
const char parameterValue2Code = 'W';
const char elementCode = 'E';
const char timeCode = 'T';
const char modeCode = 'M';
const char msgQueueCode = 'Q';

//Added for Farmbot-Sensor 
const char movingSpeedCode = 'D';
const char sensorCode    = 'K';
const char latitudeCode  = 'L';
const char longitudeCode = 'O';
const char headingCode   = 'H';
const char sysCovarianceCode = 'I';
const char measureCovarianceCode ='J';
const char matrixColCode = 'N';
const char matrixRowCode = 'U';

CommandCodeEnum commandCodeEnum = CODE_UNDEFINED;

Command::Command(char *commandChar)
{

  char *charBuf = commandChar;
  char *charPointer;
  bool invalidCommand = false;

  charPointer = strtok(charBuf, " \n\r\0");

  if (charPointer[0] == 'G' || charPointer[0] == 'F' || charPointer[0] == 'K') // added 'K' for Farmbot sensor by Jaehyun Shin
  {
    commandCodeEnum = getGCodeEnum(charPointer);
  }
  else
  {
    invalidCommand = true;
    return;
  }

  while (charPointer != NULL)
  {
    getParameter(charPointer);

    charPointer = strtok(NULL, " \n\r");
  }
}

CommandCodeEnum Command::getGCodeEnum(char *code)
{
  if (strcmp(code, "G0") == 0 || strcmp(code, "G00") == 0)
  {
    return G00;
  }
  if (strcmp(code, "G1") == 0 || strcmp(code, "G01") == 0)
  {
    return G01;
  }
    if (strcmp(code, "G28") == 0)
  {
    return G28;
  }

  //if (strcmp(code, "F3") == 0 || strcmp(code, "F03") == 0) {
  //	return F03;
  //}

  if (strcmp(code, "F09") == 0 || strcmp(code, "F9") == 0)
  {
    return F09;
  }

  if (strcmp(code, "F11") == 0)
  {
    return F11;
  }
  if (strcmp(code, "F12") == 0)
  {
    return F12;
  }
  if (strcmp(code, "F13") == 0)
  {
    return F13;
  }
  if (strcmp(code, "F14") == 0)
  {
    return F14;
  }
  if (strcmp(code, "F15") == 0)
  {
    return F15;
  }
  if (strcmp(code, "F16") == 0)
  {
    return F16;
  }

  if (strcmp(code, "F20") == 0)
  {
    return F20;
  }
  if (strcmp(code, "F21") == 0)
  {
    return F21;
  }
  if (strcmp(code, "F22") == 0)
  {
    return F22;
  }

  if (strcmp(code, "F31") == 0)
  {
    return F31;
  }
  if (strcmp(code, "F32") == 0)
  {
    return F32;
  }

  if (strcmp(code, "F41") == 0)
  {
    return F41;
  }
  if (strcmp(code, "F42") == 0)
  {
    return F42;
  }
  if (strcmp(code, "F43") == 0)
  {
    return F43;
  }
  if (strcmp(code, "F44") == 0)
  {
    return F44;
  }

  if (strcmp(code, "F61") == 0)
  {
    return F61;
  }

  if (strcmp(code, "F81") == 0)
  {
    return F81;
  }
  if (strcmp(code, "F82") == 0)
  {
    return F82;
  }
  if (strcmp(code, "F83") == 0)
  {
    return F83;
  }
  if (strcmp(code, "F84") == 0)
  {
    return F84;
  }

  // add by Jaehyun Shin
  // add 'K' for Farmbot sensor
  if (strcmp(code, "K11") == 0)
  {
    return K11;
  }
  if (strcmp(code, "K51") == 0)
  {
    return K51;
  }
  if (strcmp(code, "K61") == 0)
  {
    return K61;
  }
  if (strcmp(code, "K62") == 0)
  {
  return K62;
  }
  if (strcmp(code, "K63") == 0)
  {
  return K63;
  }
  if (strcmp(code, "K70") == 0)
  {
    return K70;
  }
   if (strcmp(code, "K71") == 0)
  {
    return K71;
  }
   if (strcmp(code, "K72") == 0)
  {
    return K72;
  }
   if (strcmp(code, "K73") == 0)
  {
    return K73;
  }
   if (strcmp(code, "K74") == 0)
  {
    return K74;
  }
   if (strcmp(code, "K75") == 0)
  {
    return K75;
  }
   if (strcmp(code, "K76") == 0)
  {
    return K76;
  }
   if (strcmp(code, "K77") == 0)
  {
    return K77;
  }
   if (strcmp(code, "K78") == 0)
  {
    return K78;
  }
   if (strcmp(code, "K79") == 0)
  {
    return K79;
  }

  return CODE_UNDEFINED;
}

double minusNotAllowed(double value)
{
  if (value < 0)
  {
    return 0;
  }
  return value;
}

void Command::getParameter(char *charPointer)
{

  //msgQueue          =  24;

  if (charPointer[0] == axisCodes[0])
  {
    axisValue[0] = atof(charPointer + 1);
    //msgQueue	= 77;
  }

  if (charPointer[0] == axisCodes[1])
  {
    axisValue[1] = atof(charPointer + 1);
  }

  if (charPointer[0] == axisCodes[2])
  {
    axisValue[2] = atof(charPointer + 1);
  }

  if (charPointer[0] == axisSpeedCodes[0])
  {
    axisSpeedValue[0] = atof(charPointer + 1);
  }

  if (charPointer[0] == axisSpeedCodes[1])
  {
    axisSpeedValue[1] = atof(charPointer + 1);
  }

  if (charPointer[0] == axisSpeedCodes[2])
  {
    axisSpeedValue[2] = atof(charPointer + 1);
  }

  if (charPointer[0] == speedCode)
  {
    speedValue = atof(charPointer + 1);
  }

  if (charPointer[0] == parameterIdCode)
  {
    parameterId = atof(charPointer + 1);
  }

  if (charPointer[0] == parameterValueCode)
  {
    parameterValue = atof(charPointer + 1);
  }

  if (charPointer[0] == parameterValue2Code)
  {
    parameterValue2 = atof(charPointer + 1);
  }

  if (charPointer[0] == elementCode)
  {
    element = atof(charPointer + 1);
  }

  if (charPointer[0] == timeCode)
  {
    time = atof(charPointer + 1);
  }

  if (charPointer[0] == modeCode)
  {
    mode = atof(charPointer + 1);
  }

  if (charPointer[0] == msgQueueCode)
  {
    msgQueue = atof(charPointer + 1);
    //msgQueue   =  5;
  }

  // add by Jaehyun Shin
  if (charPointer[0] == movingSpeedCode)
  {
    movingSpeed = atof(charPointer + 1);
  }
  
  if (charPointer[0] == sysCovarianceCode)
  {
    sysCovariance = atof(charPointer + 1);
  }

  if (charPointer[0] == measureCovarianceCode)
  {
    measureCovariance = atof(charPointer + 1);
  }

  if (charPointer[0] == matrixRowCode)
  {
    matrixIndex[0] = atof(charPointer + 1);
  }
  if (charPointer[0] == matrixColCode)
  {
    matrixIndex[1] = atof(charPointer + 1);
  }
}

void Command::print()
{
  Serial.print("R99 Command with code: ");
  Serial.print(commandCodeEnum);
  Serial.print(", X: ");
  Serial.print(axisValue[0]);
  Serial.print(", Y: ");
  Serial.print(axisValue[1]);
  Serial.print(", Z: ");
  Serial.print(axisValue[2]);
  Serial.print(", S: ");
  Serial.print(speedValue);
  Serial.print(", P: ");
  Serial.print(parameterId);
  Serial.print(", V: ");
  Serial.print(parameterValue);

  Serial.print(", W: ");
  Serial.print(parameterValue2);
  Serial.print(", T: ");
  Serial.print(time);
  Serial.print(", E: ");
  Serial.print(element);
  Serial.print(", M: ");
  Serial.print(mode);

  Serial.print(", A: ");
  Serial.print(axisSpeedValue[0]);
  Serial.print(", B: ");
  Serial.print(axisSpeedValue[1]);
  Serial.print(", C: ");
  Serial.print(axisSpeedValue[2]);

  Serial.print(", Q: ");
  Serial.print(msgQueue);

  Serial.print("\r\n");
}

CommandCodeEnum Command::getCodeEnum()
{
  return commandCodeEnum;
}

double Command::getX()
{
  return axisValue[0];
}

double Command::getY()
{
  return axisValue[1];
}

double Command::getZ()
{
  return axisValue[2];
}

long Command::getA()
{
  return axisSpeedValue[0];
}

long Command::getB()
{
  return axisSpeedValue[1];
}

long Command::getC()
{
  return axisSpeedValue[2];
}

long Command::getP()
{
  return parameterId;
}

long Command::getV()
{
  return parameterValue;
}

long Command::getW()
{
  return parameterValue2;
}

long Command::getT()
{
  return time;
}

long Command::getE()
{
  return element;
}

long Command::getM()
{
  return mode;
}

long Command::getQ()
{
  //msgQueue = 9876;
  return msgQueue;
}
long Command::getRow()
{
  return matrixIndex[0];
}
long Command::getCol()
{
  return matrixIndex[1];
}
unsigned int Command::getMS()
{
  return movingSpeed;
}
double Command::getsysCovariance()
{
  return sysCovariance;
}
double Command::getmeasureCovariance()
{
  return measureCovariance;
}