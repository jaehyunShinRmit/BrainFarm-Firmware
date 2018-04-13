/*
 * K71Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K71Handler_H_
#define K71Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K71Handler : public KCodeHandler
{
public:
  static K71Handler *getInstance();
  int execute(Command *);

private:
  K71Handler();
  K71Handler(K71Handler const &);
  void operator=(K71Handler const &);
};

#endif /* K71Handler_H_ */
