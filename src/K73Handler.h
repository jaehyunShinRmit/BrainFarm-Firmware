/*
 * K73Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K73Handler_H_
#define K73Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K73Handler : public KCodeHandler
{
public:
  static K73Handler *getInstance();
  int execute(Command *);

private:
  K73Handler();
  K73Handler(K73Handler const &);
  void operator=(K73Handler const &);
};

#endif /* K73Handler_H_ */
