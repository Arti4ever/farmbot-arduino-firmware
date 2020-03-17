// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _farmbot_arduino_controller_H_
#define _farmbot_arduino_controller_H_
#include "Arduino.h"

//add your includes for the project farmbot_arduino_controller here
#include "Board.h"
#include "pins.h"
#include "Config.h"
#include "MemoryFree.h"
#include "Debug.h"

#include "TMCStepper.h"


#include "Movement.h"
#include "ServoControl.h"
#include "PinGuard.h"
#include "CurrentState.h"
#include <SPI.h>
#include "Command.h"
#include "GCodeProcessor.h"

//add your function definitions for the project farmbot_arduino_controller here

  void setPinInputOutput();
  void startSerial();

  #if defined(BOARD_HAS_TMC2130_DRIVER)
  void loadTMC2130drivers();
  void loadTMC2130parameters();
  void startupTmc();
  #endif

  void startMotor();
  void readParameters();
  void loadMovementSetting();
  void startPinGuard();
  void startServo();
  void startInterrupt();
  void homeOnBoot();
  void setupTestForDebug();
  void runTestForDebug();
  void checkEncoders();
  void checkPinGuard();
  void checkSerialInputs();
  void checkEmergencyStop();
  void checkParamsChanged();
  void periodicChecksAndReport();
  void initLastAction();
  void checkMotorsInactivity();


//Do not add code below this line
#endif /* _farmbot_arduino_controller_H_ */
