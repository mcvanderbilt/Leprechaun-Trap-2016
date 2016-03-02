/***************************************************************************************
Name      : Arduino-Controlled Leprechaun Trap v2016.03.01.001
License   : Creative Commons Attribution-ShareAlike 3.0 License
            http://creativecommons.org/licenses/by-sa/3.0/
Created   : 03/01/2016 09:55 Matthew C. Vanderbilt (START)
            TBD (END)
/***************************************************************************************
CHANGE LOG:
03/01/2016: Created Code
/***************************************************************************************
- Knock Sensor: 
  Adapted from https://www.arduino.cc/en/Tutorial/Knock created 25 Mar 2007 by
  David Cuartielles; modified 30 Aug 2011 by Tom Igoe
/***************************************************************************************/

// THINGSHIELD - REFERENCES
#include <Wire.h>                                       // General Reference
#include <Adafruit_DRV2605.h>                           // Adafruit Haptic Feedback Controller Library
#include <Adafruit_MotorShield.h>                       // Adafruit Motor Shield Library
#include "Utility/Adafruit_MS_PWMServoDriver.h"         // Adafruit Motor Shield Library
#include <Adafruit_NeoPixel.h>                          // Adafruit Neopixel Library


// CONSTANTS - COMPILER DEFINITIONS (https://www.arduino.cc/en/Reference/Define)
/* Analog Pins:             A0, A1, A2, A3, A4, A5 {aka PINS 14 through 19}
   Digital Pins (Default):  0, 1, 13 {Rx, Tx, LED}
   Digital Pins (Non-PWM):  2, 4, 7, 8, 12
   Digital Pins (PWM):      3, 5, 6, 9, 10, 11  */
#define PIN_SENSOR_KNOCK                A0              // Knock Sensor Input Pin
#define PIN_USEDASDIGITAL_15            A1              // Analog pin utilized for digital I/O
#define PIN_SERVO_LOCKPOSITION          A2              // Pin to receive input from feedback servo
#define PIN_RANDOM_SEED                 A3              // Unused Port for Random-Seed-Number Generation
#define PIN_I2C_1                       A4              // TWI/I2C PIN
#define PIN_I2C_2                       A5              // TWI/I2C PIN
#define PIN_DEFAULT_RX                  0               // Default Receive-Transmission Pin {USB/FTDI Serial.read(); aka RX}
#define PIN_DEFAULT_TX                  1               // Default Transmit Pin {USB/FTDI Serial.print(); aka TX}
#define PIN_SENSOR_IRBEAM               2               // IR Beam Sensor Pin {aka IRQ0}
//<<TODO>> Change Neopixel RX to HIGH/LOW output to trinket, where trinket runs RED on HIGH and rainbow on LOW
#define PIN_NEOPIXEL_RX                 3               // Neopixel LED Receive Pin {aka IRQ1}
#define PIN_NEOPIXEL_TX                 4               // Neopixel LED Transmit Pin
#define PIN_LED_BAR_A                   5               // LEDBar Section A
#define PIN_LED_BAR_B                   6               // LEDBar Section B
#define PIN_LED_BAR_C                   7               // LEDBar Section C
#define PIN_SERVO_SIGNPOSITION_A        8               // Sign Position Input A
#define PIN_SERVO_SIGNPOSITION_B        9               // Sign Position Input B
#define PIN_SERVO_SIGN                  10              // Sign Servo Control
#define PIN_SERVO_LOCK                  11              // Lock Servo Control
#define PIN_SWITCH_OPENTRAP             12              // Input for Open Trap Switch
#define PIN_DEFAULT_LED                 13              // On Board LED {also LED_BUILTIN}
#define PIN_USEDASANALOG_A0             14              // Analog pin utilized for analog I/O
#define PIN_SWITCH_RANDOMEVENT          15              // Random Event Switch Pin {aka A1}
#define PIN_USEDASANALOG_A2             16              // Analog pin utilized for analog I/O
#define PIN_USEDASANALOG_A3             17              // Analog pin utilized for analog I/O
#define PIN_USEDASANALOG_A4             18              // Analog pin utilized for analog I/O
#define PIN_USEDASANALOG_A5             19              // Analog pin utilized for analog I/O
#define SERIAL_BAUD_RATE                9600            // Baud Rate for Serial Port


// CONSTANTS - CODE (https://www.arduino.cc/en/Reference/Const)
const uint8_t sensorKnockThreshold      = 100;          // Threshold Value to Determine if Knock is Detected
const uint8_t stepperMotorSteps         = 200;          // Number of Steps in Stepper Motors
const uint8_t stepperMotorRightAngle    = 50;           // 90 Degrees {90 / 360 / 200}
const uint8_t counterMasterLoopLimit    = 100;          // Point at which the master loop counter should reset
const uint8_t hapticEffects             = 8;            // Number of Waveforms Available
const int     randomCaptureEventMax     = 1000;         // High-Range for Initialization of Random Capture Event
const int     randomNoiseEventMax       = 333;          // High-Range for Initialization of Random Noise Event


// GLOBAL VARIABLE INITIALIZATION
bool          isDebugEnabled            = true;         // Enable / Disable Debugging
bool          isOpen                    = false;        // Door is Opened / Closed {?? Can this be read from the Stepper Motor??}
bool          isKnockEnabled            = false;        // Knock Sensor Enabled / Disabled
bool          isRandomEventEnabled      = false;        // Variable for holding the Random Event Switch setting
uint8_t       counterMasterLoop         = 0;            // Variable to store Master Loop counts
int           sensorKnockRead           = 0;            // Variable for Holding the Knock Sensor Read Value
int           randomCaptureEventValue   = 0;            // Variable to store Random Capture Event Enabler
int           randomCaptureEventCheck   = 0;            // Variable to store Random Capture Event Values
int           randomNoiseEventValue     = 0;            // Variable to store Random Noise Event Enabler
int           randomNoiseEventCheck     = 0;            // Variable to store Random Noise Event Values
volatile bool isLeprechaunCaptured      = false;        // Variable for holding whether or not a leprechaun was captured


// INITIALIZE SHIELD(S)
    // HAPTIC FEEDBACK CONTROLLER
    Adafruit_DRV2605 HapticFeedback;                                            // Create Haptic Feedback Controller Object
  
    // MOTOR SHIELD
    Adafruit_MotorShield AFMS = Adafruit_MotorShield();                         // Create Motor Shield Object with Default I2C Address
    Adafruit_StepperMotor *motorDoor = AFMS.getStepper(stepperMotorSteps, 1);   // Connect a Stepper Motor with 1.8-degree Steps on Port #1 (M1 and M2)

  
// ARDUINO INITIALIZATION
void setup()
{
    if (isDebugEnabled)                                                         // Initialize Serial Port for Debugging
    {
        Serial.begin(SERIAL_BAUD_RATE);
        Serial.println("setup()");
    }
    initializeRandomization();                                                  // Run Randomization Initialization Routine
    initializePins();                                                           // Run Pin Initialization Routine  
    initializeNeopixels();                                                      // Run Neopixel Initialization Routine
    initializeMotors();                                                         // Run Motor Initialization Routine
}  

/**ARDUINO PRIMARY LOGIC****************************************************************/
void loop()
{
    if (isDebugEnabled && counterMasterLoop == 1)
    {
        Serial.println("loop()");
    }
    loopCounter();                                                              // Run Loop Increment
    readSwitchOpenDoor();                                                       // Run Open Door Routine
    readSwitchRandomEvent();                                                    // Set [isRandomEventEnabled]
    sensorKnock();                                                              // Run Knock Sensor Logic
    randomEvent();                                                              // Run Random Event Routine
}
  

/**CUSTOM INITIALIZATION ROUTINES*******************************************************/
void initializePins()
{
    if (isDebugEnabled)
    {
        Serial.println("--initializePins()");
    }
  
    //Initialization of Digital Input Pins for Interrupt Service Routines (ISR)
    // Trigger sensorIRBeam() if it pulls LOW (i.e. broken beam)
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_IRBEAM), sensorIRBeam, LOW);
  
    //Initialization of Digital Input Pins
    pinMode(PIN_NEOPIXEL_RX,INPUT);
    pinMode(PIN_SERVO_SIGNPOSITION_A,INPUT);
    pinMode(PIN_SERVO_SIGNPOSITION_B,INPUT);
    pinMode(PIN_SWITCH_OPENTRAP,INPUT);
    pinMode(PIN_SWITCH_RANDOMEVENT,INPUT);
  
    //Initialization of Digital Output Pins
    pinMode(PIN_NEOPIXEL_TX,OUTPUT);
    pinMode(PIN_LED_BAR_A,OUTPUT);
    pinMode(PIN_LED_BAR_B,OUTPUT);
    pinMode(PIN_LED_BAR_C,OUTPUT);
    pinMode(PIN_DEFAULT_LED,OUTPUT);
}


void initializeRandomization()
{
    if (isDebugEnabled)
    {
        Serial.println("--InitializeRandomization()");
    }
    randomEvent();                                                              // Run Random Event Routine
    randomSeed(analogRead(PIN_RANDOM_SEED));                                    // Randomizes Randomization Table Seed
    randomCaptureEventValue = random(0,randomCaptureEventMax);                  // Initializes Figure to Enable Random Capture Event
    randomNoiseEventValue = random(0,randomNoiseEventMax);                      // Initializes Figure to Enable Random Noise Event
}


void initializeHapticFeedback()
{
    if (isDebugEnabled)
    {
        Serial.println("--initializeHapticFeedback");
    }
    HapticFeedback.begin();
    HapticFeedback.selectLibrary(1);
    HapticFeedback.setMode(DRV2605_MODE_INTTRIG);
}
  
  
void initializeMotors()
{
    if (isDebugEnabled)
    {
        Serial.println("--initializeMotors()");
    }
    AFMS.begin();                                                               // Initialize with Default Frequency of 1.6KHz
    motorDoor->setSpeed(10);                                                    // Set Motor Speed at 10 rpm
  
    // Close door to hard stop then open to 90-degrees; require if there's not a sensor to detect when the door is closed
    setDoorOpen(false);
    setDoorOpen(true);
}  


void initializeNeopixels()
{
  //TBD
}


/**SECONDARY LOGIC ROUTINES*************************************************************/
void loopCounter()
{
  if (isDebugEnabled && counterMasterLoop == 1)
  {
    Serial.println("--loopCounter()");
  }
  if (counterMasterLoop = counterMasterLoopLimit)
  {
    counterMasterLoop = 1;
  }
  else
  {
    counterMasterLoop++;
  } 
}


void randomEvent()
{
  if (isDebugEnabled && counterMasterLoop == 1)
  {
    Serial.println("--randomEvent()");
  }
  randomCaptureEventCheck = random(0,randomCaptureEventMax);                  // Initializes Figure to Enable Random Capture Event
  randomNoiseEventCheck = random(0,randomNoiseEventMax);                      // Initializes Figure to Enable Random Noise Event
  if (isOpen && randomCaptureEventCheck == randomCaptureEventValue)
  {
    actionHapticFeedback(1);
    captureLeprechaun();
  }
  else if (isLeprechaunCaptured && randomNoiseEventCheck == randomNoiseEventValue)
  {
    actionHapticFeedback(random(3,hapticEffects - 1));
  }
}


void readSwitchOpenDoor()
{
  if (isDebugEnabled && counterMasterLoop == 1)
  {
    Serial.println("--readSwitchOpenDoor()");
  }
  if (digitalRead(PIN_SWITCH_OPENTRAP) == HIGH)
  {
    setDoorOpen(true);
  }
}


void readSwitchRandomEvent()
{
  if (isDebugEnabled && counterMasterLoop == 1)
  {
    Serial.println("--readSwitchRandomEvent()");
  }
  if (digitalRead(PIN_SWITCH_RANDOMEVENT) == HIGH)
  {
    isRandomEventEnabled = true;                                                // Enable the occurence of random events
  }
  else
  {
    isRandomEventEnabled = false;                                               // Disable the occurence of random events
  }
}


void sensorIRBeam()
{
  // Interrupt trigger that should only fire while isOpen = true
  if (isDebugEnabled)
  {
    Serial.println("--sensorIRBeam");
  }
  captureLeprechaun();
}


void sensorKnock()
{ if (isDebugEnabled && counterMasterLoop == 1)
  {
    Serial.println("--sensorKnock()");
  }
  sensorKnockRead = analogRead(PIN_SENSOR_KNOCK);                               // Read and store sensor input
  if (sensorKnockRead >= sensorKnockThreshold)
  {
    if (isDebugEnabled)
    {
      Serial.print ("Knock Reading: ");
      Serial.print (sensorKnockRead);
      Serial.println();
    }
    if (isKnockEnabled)
    {
      actionHapticFeedback(0);
    }
  }
}  


/**TERTIARY ACTION ROUTINES*************************************************************/
void actionHapticFeedback(uint8_t selectedWaveform)
{
  if (isDebugEnabled)
  {
    Serial.print ("----actionHapticFeedback(");
    Serial.print (selectedWaveform);
    Serial.print (")");
    Serial.println();
  }
  loadHapticFeedbackWaveform(selectedWaveform);                                // Loads the desired waveform
  HapticFeedback.go();                                                         // Play Haptic Feedback Waveform
}  


void captureLeprechaun()
{
  if (isDebugEnabled)
  {
    Serial.print ("----captureLeprechaun()");
  }
  digitalWrite(PIN_LED_BAR_A,HIGH);
  digitalWrite(PIN_LED_BAR_B,HIGH);
  digitalWrite(PIN_LED_BAR_C,HIGH);
  waitSeconds(1/3);
  digitalWrite(PIN_LED_BAR_C,LOW);
  waitSeconds(1/3);
  digitalWrite(PIN_LED_BAR_B,LOW);
  waitSeconds(1/3);
  digitalWrite(PIN_LED_BAR_A,LOW);
  setDoorOpen(false);                                                          // Close door and disable interrupts
  isLeprechaunCaptured = true;
  //<<TODO>> ENGAGE LOCK
  //<<TODO>> CHANGE LIGHTING
  //<<TODO>> ROTATE SIGN
}


void setDoorOpen(bool isDoorOpen)
{
  if (isDebugEnabled)
  {
    Serial.print ("----setDoorStatus(");
    Serial.print (isDoorOpen);
    Serial.print (")");
    Serial.println();
  }
  isOpen = isDoorOpen;
  if (isOpen)
  {
    motorDoor->step(stepperMotorRightAngle, FORWARD, MICROSTEP);               // Open Door to 90-Degree Angle
    if (isLeprechaunCaptured)
    {
      actionHapticFeedback(2);
      isLeprechaunCaptured = false;
    }
    interrupts();                                                              // Enables Interrupt Actions
  }
  else
  {
    noInterrupts();                                                            // Disables Interrupt Actions
    motorDoor->step(2*stepperMotorRightAngle, BACKWARD, MICROSTEP);            // Close Door Hard Stop
  }
}
  
  
/**QUATERNARY ACTION ROUTINES***********************************************************/  
void loadHapticFeedbackWaveform(uint8_t loadSelectedWaveform)
{
  if (isDebugEnabled)
  {
    Serial.print ("------loadHapticFeedbackWaveform(");
    Serial.print (loadSelectedWaveform);
    Serial.print (")");
    Serial.println();
  }
  
  /* Load Haptic Feedback Waveforms:
     0 = Knock
     1 = Sneak Inside
     2 = Run Output
     3 = Inside Option 1
     4 = Inside Option 2
     5 = Inside Option 3
     6 = Inside Option 4
     7 = Inside Option 5 */
  static const uint8_t hapticEffect[hapticEffects][7] = { { 7, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 },
                                                          { 0, 0, 0, 0, 0, 0, 0 } };
  
  // Load Selected Waveform to Controller
  for (int i = 0; i < hapticEffects; i++)
  {
    HapticFeedback.setWaveform(i, hapticEffect[loadSelectedWaveform][i]);
  }
}

void waitSeconds(float seconds)
{
  if (isDebugEnabled)
  {
    Serial.println("------waitSeconds(");
  }
    const int milliseconds= 1000;
    delay(seconds * milliseconds);
}
