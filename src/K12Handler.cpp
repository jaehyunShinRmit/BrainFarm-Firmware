/*
 * K12Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#include "K12Handler.h"

static K12Handler *instance;

K12Handler *K12Handler::getInstance()
{
  if (!instance)
  {
    instance = new K12Handler();
  };
  return instance;
};

K12Handler::K12Handler()
{
}

int K12Handler::execute(Command *command)
{
  Serial.print("K12 Advance the robot and Data LOGGING start \r\n");

  KCurrentState::getInstance()->updateFileName();
  Serial.println("File name updated");
  KCurrentState::getInstance()->printHeader();
  Serial.println("Header printed");

  KCurrentState::getInstance()->setMovingDistance(command->getMD());
  KCurrentState::getInstance()->setMovingDirection(ADVANCE);

  KCurrentState::getInstance()->setisMoving(true);
  Serial.println("Start Moving");
  KCurrentState::getInstance()->setisRawdataLogging(true);

 
  return 0;
}
