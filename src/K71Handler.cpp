/*
 * K71Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K71Handler.h"

static K71Handler *instance;

K71Handler *K71Handler::getInstance()
{
  if (!instance)
  {
    instance = new K71Handler();
  };
  return instance;
};

K71Handler::K71Handler()
{
}

int K71Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Updating robot initial start position \r\n");
  }

  KCurrentState::getInstance()->setisUpdatingReferenceLocation(true);

  return 0;
}
