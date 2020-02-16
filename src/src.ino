/*
 Name:		src.ino
 Created:	11/14/2019 9:51:10 PM
 Author:	Tim Evers
*/

#include "farmbot_arduino_controller.h"

// the setup function runs once when you press reset or power the board
void setup()
{
  startSerial();
  setPinInputOutput();

  readParameters();

#if defined(BOARD_HAS_TMC2130_DRIVER)
  loadTMC2130drivers();
  startupTmc();
  loadTMC2130parameters();
#endif

  loadMovementSetting();
  startMotor();
  startPinGuard();
  //startServo();
  startInterrupt();
  initLastAction();
  homeOnBoot();

  //setupTestForDebug();

  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("ARDUINO STARTUP COMPLETE");
  Serial.print(CRLF);
}

// the loop function runs over and over again until power down or reset
void loop()
{
  //runTestForDebug();

  checkEncoders();
  checkPinGuard();
  checkSerialInputs();
  checkEmergencyStop();
  checkParamsChanged();
  periodicChecksAndReport();

}
