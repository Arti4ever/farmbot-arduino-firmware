
/*
 * F13Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 */

#include "F13Handler.h"

static F13Handler *instance;

F13Handler *F13Handler::getInstance()
{
  if (!instance)
  {
    instance = new F13Handler();
  };
  return instance;
};

F13Handler::F13Handler()
{
}

int F13Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 HOME Z\r\n");
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
  double xPos = (double)CurrentState::getInstance()->getX() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_X);
  double yPos = (double)CurrentState::getInstance()->getY() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_Y);

  // Move to home position. Then 1 times move away and move to home again.
  for (int stepNr = 0; stepNr < 3; stepNr++)
  {
    switch (stepNr)
    {
    case 0: Movement::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
    case 1: Movement::getInstance()->moveToCoords(xPos, yPos, moveAwayCoord, 0, 0, 0, false, false, false); break;
    case 2: Movement::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
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
