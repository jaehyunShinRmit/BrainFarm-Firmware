/*
 * K79Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K79Handler_H_
#define K79Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K79Handler : public KCodeHandler
{
public:
  static K79Handler *getInstance();
  int execute(Command *);

private:
  K79Handler();
  K79Handler(K79Handler const &);
  void operator=(K79Handler const &);
};

#endif /* K79Handler_H_ */
