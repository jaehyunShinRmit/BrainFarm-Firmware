/*
 * K17Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 4/05/2018
 */

#ifndef K17Handler_H_
#define K17Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K17Handler : public KCodeHandler
{
public:
  static K17Handler *getInstance();
  int execute(Command *);

private:
  K17Handler();
  K17Handler(K17Handler const &);
  void operator=(K17Handler const &);
};

#endif /* K17Handler_H_ */
