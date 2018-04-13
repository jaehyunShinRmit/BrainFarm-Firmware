/*
 * K61Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef K61HANDLER_H_
#define K61HANDLER_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K61Handler : public KCodeHandler
{
public:
  static K61Handler *getInstance();
  int execute(Command *);

private:
  K61Handler();
  K61Handler(K61Handler const &);
  void operator=(K61Handler const &);
};

#endif /* K61HANDLER_H_ */
