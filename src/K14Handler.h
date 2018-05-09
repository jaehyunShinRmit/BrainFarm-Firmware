/*
 * K14Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#ifndef K14Handler_H_
#define K14Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K14Handler : public KCodeHandler
{
public:
  static K14Handler *getInstance();
  int execute(Command *);

private:
  K14Handler();
  K14Handler(K14Handler const &);
  void operator=(K14Handler const &);
};

#endif /* K12Handler_H_ */
