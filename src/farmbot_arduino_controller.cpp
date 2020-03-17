// Do not remove the include below
#include "farmbot_arduino_controller.h"

#if !defined(BOARD_HAS_TMC2130_DRIVER)
#include "TimerOne.h"
#endif

bool stepperInit = false;
bool stepperFlip = false;

char commandChar[INCOMING_CMD_BUF_SIZE + 1];
//String commandString = "";
char incomingChar = 0;
char incomingCommandArray[INCOMING_CMD_BUF_SIZE];
int incomingCommandPointer = 0;
static GCodeProcessor *gCodeProcessor = new GCodeProcessor();

unsigned long reportingPeriod = 5000;
unsigned long lastAction;
bool previousEmergencyStop = false;

unsigned long pinGuardLastCheck;

int lastParamChangeNr = 0;

// Blink led routine used for testing
bool blink = false;
void blinkLed()
{
  blink = !blink;
  digitalWrite(LED_PIN, blink);
}

// Interrupt handling for:
//   - movement
//   - encoders
//   - pin guard
//

unsigned long interruptStartTime = 0;
unsigned long interruptStopTime = 0;
unsigned long interruptDuration = 0;
unsigned long interruptDurationMax = 0;

bool interruptBusy = false;
int interruptSecondTimer = 0;

#if !defined(BOARD_HAS_TMC2130_DRIVER)
void interrupt(void)
{
  if (!debugInterrupt)
  {
    //interruptSecondTimer++;

    if (interruptBusy == false)
    {
      //interruptStartTime = micros();

      interruptBusy = true;
      Movement::getInstance()->handleMovementInterrupt();
      interruptBusy = false;
    }
  }
}
#else
ISR(TIMER2_OVF_vect) 
{
  if (interruptBusy == false)
  {
    interruptBusy = true;
    Movement::getInstance()->handleMovementInterrupt();
    interruptBusy = false;
  }
}
#endif

void checkPinGuard()
{
 // Check PinGuards
  if (millis() - pinGuardLastCheck >= 1000)
  {
    // check the pins for timeouts
    PinGuard::getInstance()->checkPins();
    pinGuardLastCheck = millis();
  }
}

void periodicChecksAndReport()
{
  // Do periodic checks and feedback
  if ((millis() - lastAction) > reportingPeriod)
  {
    // After an idle time, send the idle message

    if (CurrentState::getInstance()->isEmergencyStop())
    {
      Serial.print(COMM_REPORT_EMERGENCY_STOP);
      CurrentState::getInstance()->printQAndNewLine();

      if (debugMessages)
      {
        Serial.print(COMM_REPORT_COMMENT);
        Serial.print(" Emergency stop engaged");
        CurrentState::getInstance()->printQAndNewLine();
      }
    }
    else
    {
      Serial.print(COMM_REPORT_CMD_IDLE);
      CurrentState::getInstance()->printQAndNewLine();
    }

    Movement::getInstance()->storePosition();
    CurrentState::getInstance()->printPosition();

    #if defined(BOARD_HAS_ENCODER)
      Movement::getInstance()->reportEncoders();
    #endif

    CurrentState::getInstance()->storeEndStops();
    CurrentState::getInstance()->printEndStops();

    if (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) != 1)
    {
      Serial.print(COMM_REPORT_NO_CONFIG);
      CurrentState::getInstance()->printQAndNewLine();
    }

    lastAction = millis();
  }
}

void checkSerialInputs()
{
  if (Serial.available())
  {
    // Save current time stamp for timeout actions
    lastAction = millis();

    // Get the input and start processing on receiving 'new line'
    incomingChar = Serial.read();

    // Filter out emergency stop.
    if (!(incomingChar == 'E' || incomingChar == 'e'))
    {
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
    }
    else
    {
      CurrentState::getInstance()->setEmergencyStop();
    }

    // If the string is getting to long, cap it off with a new line and let it process anyway
    if (incomingCommandPointer >= INCOMING_CMD_BUF_SIZE - 1)
    {
      incomingChar = '\n';
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
    }

    if (incomingChar == '\n' || incomingCommandPointer >= INCOMING_CMD_BUF_SIZE)
    {
      for (int i = 0; i < incomingCommandPointer - 1; i++)
      {
          commandChar[i] = incomingCommandArray[i];
      }
      commandChar[incomingCommandPointer-1] = '\0';

      if (incomingCommandPointer > 1)
      {

        // Report back the received command
        Serial.print(COMM_REPORT_CMD_ECHO);
        Serial.print(" ");
        Serial.print("*");
        Serial.print(commandChar);
        Serial.print("*");
        Serial.print("\r\n");

        // Create a command and let it execute
        Command *command = new Command(commandChar);

        // Log the values if needed for debugging
        if (LOGGING || debugMessages)
        {
          command->print();
        }

        gCodeProcessor->execute(command);

        free(command);

      }
      incomingCommandPointer = 0;
    }
  }
}

void checkEmergencyStop()
{
  // In case of emergency stop, disable movement and
  // shut down the pins used
  if (previousEmergencyStop == false && CurrentState::getInstance()->isEmergencyStop())
  {
    Movement::getInstance()->disableMotorsEmergency();
    PinControl::getInstance()->resetPinsUsed();
    ServoControl::getInstance()->detachServos();
    if (debugMessages)
    {
      Serial.print(COMM_REPORT_COMMENT);
      Serial.print(" Going to safe state");
      CurrentState::getInstance()->printQAndNewLine();
    }
  }
  previousEmergencyStop = CurrentState::getInstance()->isEmergencyStop();
}

void checkParamsChanged()
{
  // Check if parameters are changed, and if so load the new settings
  if (lastParamChangeNr != ParameterList::getInstance()->paramChangeNumber())
  {
    lastParamChangeNr = ParameterList::getInstance()->paramChangeNumber();

    if (debugMessages)
    {
      Serial.print(COMM_REPORT_COMMENT);
      Serial.print(" loading parameters");
      CurrentState::getInstance()->printQAndNewLine();
    }

    Movement::getInstance()->loadSettings();
    PinGuard::getInstance()->loadConfig();
  }
}

void checkEncoders()
{
  
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    // Check encoders out of interrupt for farmduino 1.4
    Movement::getInstance()->checkEncoders();
  #endif
  
}


// Set pins input output
void setPinInputOutput(){
    //setup stepper drivers pins
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(X_ENABLE_PIN, OUTPUT);

    pinMode(E_STEP_PIN, OUTPUT);
    pinMode(E_DIR_PIN, OUTPUT);
    pinMode(E_ENABLE_PIN, OUTPUT);
    
    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);

    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);

    //setup endstops pins
    pinMode(X_MIN_PIN, INPUT_PULLUP);
    pinMode(X_MAX_PIN, INPUT_PULLUP);

    pinMode(Y_MIN_PIN, INPUT_PULLUP);
    pinMode(Y_MAX_PIN, INPUT_PULLUP);

    pinMode(Z_MIN_PIN, INPUT_PULLUP);
    pinMode(Z_MAX_PIN, INPUT_PULLUP);

    //setup aux stepper drivers pins
    pinMode(AUX_STEP_PIN, OUTPUT);
    pinMode(AUX_DIR_PIN, OUTPUT);
    pinMode(AUX_ENABLE_PIN, OUTPUT);
    digitalWrite(AUX_ENABLE_PIN, HIGH);

    #if defined(FYSETC_F6)
      pinMode(AUX_2_STEP_PIN, OUTPUT);
      pinMode(AUX_2_DIR_PIN, OUTPUT);
      pinMode(AUX_2_ENABLE_PIN, OUTPUT);
      digitalWrite(AUX_2_ENABLE_PIN, HIGH);    
    #endif

    #if defined(BOARD_HAS_TMC2130_DRIVER)
      pinMode(X_CHIP_SELECT, OUTPUT);
      pinMode(E_CHIP_SELECT, OUTPUT);
      pinMode(Y_CHIP_SELECT, OUTPUT);
      pinMode(Z_CHIP_SELECT, OUTPUT);
      SPI.begin();
    #endif

    //setup peripherals pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(WATER_PIN, OUTPUT);
    pinMode(LIGHTING_PIN, OUTPUT);
    pinMode(PERIPHERAL_4_PIN, OUTPUT);
    pinMode(PERIPHERAL_5_PIN, OUTPUT);

  #ifdef RAMPS_V14
    // Setup pin input/output settings
    pinMode(HEATER_0_PIN, OUTPUT);
    pinMode(HEATER_1_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);   
  #endif

  #if defined(FYSETC_F6)
    pinMode(PERIPHERAL_6_PIN, OUTPUT);
    pinMode(PERIPHERAL_7_PIN, OUTPUT);
    pinMode(RGB_LED_R_PIN, OUTPUT);
    pinMode(RGB_LED_G_PIN, OUTPUT);
    pinMode(RGB_LED_B_PIN, OUTPUT);
  #endif

    //setup UTM pins
    pinMode(UTM_C, INPUT_PULLUP);
    pinMode(UTM_D, INPUT_PULLUP);
    if (UTM_E > 0) { pinMode(UTM_E, INPUT_PULLUP); };
    if (UTM_F > 0) { pinMode(UTM_F, INPUT_PULLUP); };
    if (UTM_G > 0) { pinMode(UTM_G, INPUT_PULLUP); };
    if (UTM_H > 0) { pinMode(UTM_H, INPUT_PULLUP); };
    if (UTM_I > 0) { pinMode(UTM_I, INPUT_PULLUP); };
    if (UTM_J > 0) { pinMode(UTM_J, INPUT_PULLUP); };
    if (UTM_K > 0) { pinMode(UTM_K, INPUT_PULLUP); };
    if (UTM_L > 0) { pinMode(UTM_L, INPUT_PULLUP); };

    //setup servos pins
    pinMode(SERVO_0_PIN, OUTPUT);
    pinMode(SERVO_1_PIN, OUTPUT);
    pinMode(SERVO_2_PIN, OUTPUT);
    pinMode(SERVO_3_PIN, OUTPUT);

    //setup encoders pins
#if defined(BOARD_HAS_ENCODER)
    pinMode(X_ENCDR_A, INPUT_PULLUP);
    pinMode(X_ENCDR_B, INPUT_PULLUP);

    pinMode(Y_ENCDR_A, INPUT_PULLUP);
    pinMode(Y_ENCDR_B, INPUT_PULLUP);

    pinMode(Z_ENCDR_A, INPUT_PULLUP);
    pinMode(Z_ENCDR_B, INPUT_PULLUP);

  #ifdef RAMPS_V14
    pinMode(X_ENCDR_A_Q, INPUT_PULLUP);
    pinMode(X_ENCDR_B_Q, INPUT_PULLUP);

    pinMode(Y_ENCDR_A_Q, INPUT_PULLUP);
    pinMode(Y_ENCDR_B_Q, INPUT_PULLUP);

    pinMode(Z_ENCDR_A_Q, INPUT_PULLUP);
    pinMode(Z_ENCDR_B_Q, INPUT_PULLUP);
  #endif
#endif

#if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)

  reportingPeriod = 500;

  pinMode(READ_ENA_PIN, INPUT_PULLUP);
  pinMode(NSS_PIN, OUTPUT);
  digitalWrite(NSS_PIN, HIGH);

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.begin();

#endif

  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Set input/output");
  Serial.print(CRLF);
}

// other initialisation functions
void startSerial()
{
  Serial.begin(115200);
  delay(100);
  while (!Serial);

  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Serial connection started");
  Serial.print(CRLF);
}

void startInterrupt()
{
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Start interrupt");
  Serial.print(CRLF);

  // Start the interrupt used for moving
  // Interrupt management code library written by Paul Stoffregen
  // The default time 100 micro seconds

  #if !defined(BOARD_HAS_TMC2130_DRIVER)
    Timer1.attachInterrupt(interrupt);
    Timer1.initialize(MOVEMENT_INTERRUPT_SPEED);
    Timer1.start();
  #else
    TIMSK2 = (TIMSK2 & B11111110) | 0x01; // Enable timer overflow
    TCCR2B = (TCCR2B & B11111000) | 0x01; // Set divider to 1
    OCR2A = 4; // Set overflow to 4 for total of 64 �s    
  #endif
}

void homeOnBoot()
{
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Check homing on boot");
  Serial.print(CRLF);

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_Z) == 1)
  )
  {
    Serial.print("R99 HOME Z ON STARTUP\r\n");
    Movement::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, false, false, true);
  }

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_Y) == 1)
  )
  {
    Serial.print("R99 HOME Y ON STARTUP\r\n");
    Movement::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, false, true, false);
  }

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_X) == 1)
  )
  {
    Serial.print("R99 HOME X ON STARTUP\r\n");
    Movement::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, true, false, false);
  }
}

void startMotor()
{
  // Start the motor handling
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Set motor enables off");
  Serial.print(CRLF);

  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(E_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);
}

void loadMovementSetting()
{

  // Load motor settings
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Load movement settings");
  Serial.print(CRLF);

  Movement::getInstance()->loadSettings();
}

void readParameters()
{
  // Dump all values to the serial interface
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Read parameters");
  Serial.print(CRLF);

  ParameterList::getInstance()->readAllValues();
}

void startPinGuard()
{
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Start pin guard");
  Serial.print(CRLF);

  // Get the settings for the pin guard
  PinGuard::getInstance()->loadConfig();

  pinGuardLastCheck = millis();
}

void startServo()
{

  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Start servo");
  Serial.print(CRLF);

  ServoControl::getInstance()->attach();
}

#if defined(BOARD_HAS_TMC2130_DRIVER)
void startupTmc()
{
  // Initialize the drivers
  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Init TMC2130 drivers");
  Serial.print(CRLF);

  Movement::getInstance()->initTMC2130();
}
#endif

void initLastAction()
{
  // Initialize the inactivity check
  lastAction = millis();
}

void setupTestForDebug()
{

  Serial.print(COMM_REPORT_COMMENT);
  Serial.print(SPACE);
  Serial.print("Setup Debug");
  Serial.print(CRLF);

}

void runTestForDebug()
{

}

void checkMotorsInactivity()
{
  //if we do not move since 5min, then disable motors
  Movement::getInstance()->checkInactivity();
}
