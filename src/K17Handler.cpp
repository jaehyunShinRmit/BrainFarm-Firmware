/*
 * K17Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 04/05/2018
 */

#include "K17Handler.h"

static K17Handler *instance;

K17Handler *K17Handler::getInstance()
{
  if (!instance)
  {
    instance = new K17Handler();
  };
  return instance;
};

K17Handler::K17Handler()
{
}

int K17Handler::execute(Command *command)
{
 
  Serial.print("K17 Start Raw data logging \r\n");
  
  KCurrentState::getInstance()->updateFileName();
  Serial.println("File name updated");
  KCurrentState::getInstance()->printRawHeader();
  Serial.println("Header printed");


  KCurrentState::getInstance()->setisRawdataLogging(true);


  return 0;
}
