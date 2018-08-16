/*
 * KCodeProcessor.h
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Author: Tim Evers
 *      Modified by: Jaehyun Shin 20/02/2018
 */

#ifndef KCODEPROCESSOR_H_
#define KCODEPROCESSOR_H_

#include "Command.h"
#include "Config.h"
#include "Debug.h"

#include "KCodeHandler.h"
#include "K10Handler.h"
#include "K11Handler.h"
#include "K12Handler.h"
#include "K13Handler.h"
#include "K14Handler.h"
#include "K15Handler.h"
#include "K16Handler.h"
#include "K17Handler.h"
#include "K51Handler.h"
#include "K61Handler.h"
#include "K62Handler.h"
#include "K63Handler.h"
#include "K70Handler.h"
#include "K71Handler.h"
#include "K72Handler.h"
#include "K73Handler.h"
#include "K74Handler.h"
#include "K75Handler.h"
#include "K76Handler.h"
#include "K77Handler.h"
#include "K78Handler.h"
#include "K79Handler.h"
#include "ParameterList.h"

class KCodeProcessor
{
public:
  KCodeProcessor();
  virtual ~KCodeProcessor();
  int execute(Command *command);

protected:
  KCodeHandler *getKCodeHandler(CommandCodeEnum);

private:
  void printCommandLog(Command *);
};

#endif /* KCODEPROCESSOR_H_ */
