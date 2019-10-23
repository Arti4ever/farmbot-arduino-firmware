// Do not remove the include below
#include "farmbot_arduino_controller.h"
#include "pins.h"
#include "Config.h"
#include "StepperControl.h"
#include "ServoControl.h"
#include "PinGuard.h"
#include "MemoryFree.h"
#include "Debug.h"
#include "CurrentState.h"
#include <SPI.h>

#if !defined(BOARD_HAS_TMC2130_DRIVER)
#include "TimerOne.h"
#endif

bool stepperInit = false;
bool stepperFlip = false;

static GCodeProcessor *gCodeProcessor = new GCodeProcessor();

unsigned long reportingPeriod = 5000;
unsigned long lastAction;
unsigned long currentTime;
unsigned long cycleCounter = 0;
bool previousEmergencyStop = false;

unsigned long pinGuardLastCheck;
unsigned long pinGuardCurrentTime;

int lastParamChangeNr = 0;

String commandString = "";
char incomingChar = 0;
char incomingCommandArray[INCOMING_CMD_BUF_SIZE];
int incomingCommandPointer = 0;

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
      StepperControl::getInstance()->handleMovementInterrupt();
      interruptBusy = false;
    }
  }
}
#else
ISR(TIMER2_OVF_vect) {

  if (interruptBusy == false)
  {
    interruptBusy = true;
    StepperControl::getInstance()->handleMovementInterrupt();
    interruptBusy = false;
  }
}
#endif

//The setup function is called once at startup of the sketch
void setup()
{
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

    #if defined(BOARD_HAS_TMC2130_DRIVER)
      pinMode(X_CHIP_SELECT, OUTPUT);
      pinMode(E_CHIP_SELECT, OUTPUT);
      pinMode(Y_CHIP_SELECT, OUTPUT);
      pinMode(Z_CHIP_SELECT, OUTPUT);
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
#if defined(STEPPER_HAS_ENCODER)
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

  //disable steppers
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(E_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);

  Serial.begin(115200);
  delay(100);

  // Start the motor handling
  //ServoControl::getInstance()->attach();

  // Load motor settings
  StepperControl::getInstance()->loadSettings();

  // Dump all values to the serial interface
  // ParameterList::getInstance()->readAllValues();

  // Get the settings for the pin guard
  PinGuard::getInstance()->loadConfig();
  pinGuardLastCheck = millis();

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

  // Initialize the inactivity check
  lastAction = millis();

  pinGuardCurrentTime = millis();
  pinGuardLastCheck = millis();

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_Z) == 1)
  )
  {
    Serial.print("R99 HOME Z ON STARTUP\r\n");
    StepperControl::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, false, false, true);
  }

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_Y) == 1)
  )
  {
    Serial.print("R99 HOME Y ON STARTUP\r\n");
    StepperControl::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, false, true, false);
  }

  if
  (
    (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) == 1) &&
    (ParameterList::getInstance()->getValue(MOVEMENT_HOME_AT_BOOT_X) == 1)
  )
  {
    Serial.print("R99 HOME X ON STARTUP\r\n");
    StepperControl::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, true, false, false);
  }

  Serial.print("R99 ARDUINO STARTUP COMPLETE\r\n");

  //StepperControl::getInstance()->test2();
}

char commandChar[INCOMING_CMD_BUF_SIZE + 1];

// int cycleCountTest = 0;

// The loop function is called in an endless loop
void loop()
{

  //Serial.print(millis());
  //Serial.print("\r\n");
  //delay(1000);

  //StepperControl::getInstance()->test();
  //delayMicroseconds(100);

  //digitalWrite(LED_PIN, true);
  //delay(250);
  //digitalWrite(LED_PIN, false);
  //delay(250);

  if (debugInterrupt)
  {
    StepperControl::getInstance()->handleMovementInterrupt();
  }

  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    // Check encoders out of interrupt for farmduino 1.4
    StepperControl::getInstance()->checkEncoders();
  #endif

  pinGuardCurrentTime = millis();
  if (pinGuardCurrentTime < pinGuardLastCheck)
  {
    pinGuardLastCheck = pinGuardCurrentTime;
  }
  else
  {
    if (pinGuardCurrentTime - pinGuardLastCheck >= 1000)
    {
      pinGuardLastCheck += 1000;

      // check the pins for timeouts
      PinGuard::getInstance()->checkPins();
    }
  }

  if (Serial.available())
  {
    // Save current time stamp for timeout actions
    lastAction = millis();

    // Get the input and start processing on receiving 'new line'
    incomingChar = Serial.read();

    // Filter out emergency stop.
    if (!(incomingChar == 69 || incomingChar == 101))
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

      //char commandChar[incomingCommandPointer + 1];
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

  // In case of emergency stop, disable movement and
  // shut down the pins used
  if (previousEmergencyStop == false && CurrentState::getInstance()->isEmergencyStop())
  {
    StepperControl::getInstance()->disableMotorsEmergency();
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

    StepperControl::getInstance()->loadSettings();
    PinGuard::getInstance()->loadConfig();
  }

  // Do periodic checks and feedback

  currentTime = millis();
  if (currentTime < lastAction)
  {

    // If the device timer overruns, reset the idle counter
    lastAction = millis();
  }
  else
  {

    if ((currentTime - lastAction) > reportingPeriod)
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

      StepperControl::getInstance()->storePosition();
      CurrentState::getInstance()->printPosition();

      StepperControl::getInstance()->reportEncoders();

      CurrentState::getInstance()->storeEndStops();
      CurrentState::getInstance()->printEndStops();

      // cycleCountTest++;
      // Serial.print("R99 TST Cycle count ");
      // Serial.print(cycleCountTest);
      // Serial.print(" ");
      // CurrentState::getInstance()->printQAndNewLine();


      if (debugMessages)
      {
        Serial.print(COMM_REPORT_COMMENT);
        Serial.print(" MEM ");
        Serial.print(freeMemory());
        CurrentState::getInstance()->printQAndNewLine();

        Serial.print(COMM_REPORT_COMMENT);
        Serial.print(" IND DUR ");
        Serial.print(interruptDuration);
        Serial.print(" MAX ");
        Serial.print(interruptDurationMax);
        CurrentState::getInstance()->printQAndNewLine();

        StepperControl::getInstance()->test();
      }

      if (ParameterList::getInstance()->getValue(PARAM_CONFIG_OK) != 1)
      {
        Serial.print(COMM_REPORT_NO_CONFIG);
        CurrentState::getInstance()->printQAndNewLine();
      }

      cycleCounter++;
      lastAction = millis();
    }
  }

}
