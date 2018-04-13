/*
 * K63Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef K63Handler_H_
#define K63Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K63Handler : public KCodeHandler
{
public:
  static K63Handler *getInstance();
  int execute(Command *);

private:
  K63Handler();
  K63Handler(K63Handler const &);
  void operator=(K63Handler const &);
};

#endif /* K63Handler_H_ */
