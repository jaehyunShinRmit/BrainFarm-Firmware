/*
 * K16Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 04/05/2018
 */

#include "K16Handler.h"

static K16Handler *instance;

K16Handler *K16Handler::getInstance()
{
  if (!instance)
  {
    instance = new K16Handler();
  };
  return instance;
};

K16Handler::K16Handler()
{
}

int K16Handler::execute(Command *command)
{
 
  Serial.print("K16 Start data Logging for Reinforced Learning \r\n");
 
  KCurrentState::getInstance()->setisReinforceddataLogging(true);

  KCurrentState::getInstance()->updateFileName();
  KCurrentState::getInstance()->printHeader();
  return 0;
}
