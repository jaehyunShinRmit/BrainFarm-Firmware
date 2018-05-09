/*
 * K13Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#include "K13Handler.h"

static K13Handler *instance;

K13Handler *K13Handler::getInstance()
{
  if (!instance)
  {
    instance = new K13Handler();
  };
  return instance;
};

K13Handler::K13Handler()
{
}

int K13Handler::execute(Command *command)
{
  Serial.print("K13 Retreat the robot \r\n");

 
  KCurrentState::getInstance()->setMovingDistance(command->getMD());
  KCurrentState::getInstance()->setMovingDirection(RETREAT);
  KCurrentState::getInstance()->setisMoving(true);
  return 0;
}
