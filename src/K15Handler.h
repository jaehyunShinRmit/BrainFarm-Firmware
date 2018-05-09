/*
 * K15Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#ifndef K15Handler_H_
#define K15Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K15Handler : public KCodeHandler
{
public:
  static K15Handler *getInstance();
  int execute(Command *);

private:
  K15Handler();
  K15Handler(K15Handler const &);
  void operator=(K15Handler const &);
};

#endif /* K12Handler_H_ */
