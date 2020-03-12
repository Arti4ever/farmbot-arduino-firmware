
/*
 * F11Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 */

#include "F11Handler.h"

static F11Handler *instance;

F11Handler *F11Handler::getInstance()
{
  if (!instance)
  {
    instance = new F11Handler();
  };
  return instance;
};

F11Handler::F11Handler()
{
}

int F11Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 HOME X\r\n");
  }

  int homeIsUp = ParameterList::getInstance()->getValue(MOVEMENT_HOME_UP_X);
  int moveAwayCoord = 10;
  int execution;
  bool emergencyStop;

  if (homeIsUp == 1)
  {
    moveAwayCoord = -moveAwayCoord;
  }

  // save current position
  double yPos = (double)CurrentState::getInstance()->getY() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_Y);
  double zPos = (double)CurrentState::getInstance()->getZ() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_Z);

  // Move to home position. Then 1 times move away and move to home again.
  for (int stepNr = 0; stepNr < 3; stepNr++)
  {
    switch (stepNr)
    {
      case 0: Movement::getInstance()->moveToCoords(0, yPos, zPos, 0, 0, 0, true, false, false); break;
      case 1: Movement::getInstance()->moveToCoords(moveAwayCoord, yPos, zPos, 0, 0, 0, false, false, false); break;
      case 2: Movement::getInstance()->moveToCoords(0, yPos, zPos, 0, 0, 0, true, false, false); break;
    }

    execution = CurrentState::getInstance()->getLastError();
    emergencyStop = CurrentState::getInstance()->isEmergencyStop();

    if (emergencyStop || execution != 0)
    {
      break;
    }
  }

  if (LOGGING)
  {
    CurrentState::getInstance()->print();
  }
  return 0;
}
