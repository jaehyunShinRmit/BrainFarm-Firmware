/*
 * K72Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K72Handler_H_
#define K72Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K72Handler : public KCodeHandler
{
public:
  static K72Handler *getInstance();
  int execute(Command *);

private:
  K72Handler();
  K72Handler(K72Handler const &);
  void operator=(K72Handler const &);
};

#endif /* K72Handler_H_ */
