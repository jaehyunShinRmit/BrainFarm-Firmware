/*
 * K74Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K74Handler_H_
#define K74Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K74Handler : public KCodeHandler
{
public:
  static K74Handler *getInstance();
  int execute(Command *);

private:
  K74Handler();
  K74Handler(K74Handler const &);
  void operator=(K74Handler const &);
};

#endif /* K74Handler_H_ */
