/*
 * K14Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#include "K14Handler.h"

static K14Handler *instance;

K14Handler *K14Handler::getInstance()
{
  if (!instance)
  {
    instance = new K14Handler();
  };
  return instance;
};

K14Handler::K14Handler()
{
}

int K14Handler::execute(Command *command)
{
  Serial.print("K14 Trun Left the robot\r\n");


  KCurrentState::getInstance()->setMovingDistance(command->getMD());
  KCurrentState::getInstance()->setMovingDirection(LEFT);
  KCurrentState::getInstance()->setisMoving(true);
  return 0;
}
