/*
  Sousvido
  Jevon Wild < github (at) jevonwild (dot) com >

  It's a sous vide cooker, in Arduino! Some parts based heavily on Adafruit's
  Sous Viduino <https://github.com/adafruit/Sous_Viduino>.

  Uses a potentiometer know for setting temperature, and relies entirely on the
  autotune library to tune the PID controller. This is for applications without
  a consistent container or heating elements.

*/

// Libraries for the PID control system and autotune
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the digital thermocouple
#include <OneWire.h>
#include <DallasTemperature.h>

// WAY more readable than using the Timer/Counter Control Register directly
#include <MsTimer2.h>

// Math is FUN! But srsly, let's use a library to handle it.
#include <math.h>



//  ===================
//  = Pin Definitions =
//  ===================
#define PotentiometerPin A0
#define  ThermocouplePin 2
#define     HeaterLEDPin 3
#define        HeaterPin 4
#define        ButtonPin 5



//  ===========================
//  = Settings & Initializers =
//  ===========================
bool displayFahrenheit = true; // Set to true for F, false for C

// Pause Button buffer
bool buttonBuffer = 400; // tweak this if the pause button is acting up

// State Machine
//  Paused: The initial state, when nothing is happening. Toggle into or out of
//          this state by hitting the Pause button.
// Priming: The initial warming state, before we've reached a tunable temp.
// Resting: A cooling state, for when the temperature is way too high (or the
//          target temp has been decreased significantly).
//  Tuning: Autotuning is happening: leave things alone until it's finished.
// Running: The standard operating state. PID is in control.
enum State { PAUSED, PRIMING, RESTING, TUNING, RUNNING };
State currState = PAUSED; // Setting the initial state
State prevState = PRIMING; // Setting the state after unpausing
long stateChangedAt = 0;

// Thermocouple
OneWire oneWire(ThermocouplePin);
DallasTemperature sensors(&oneWire);
DeviceAddress waterThermocouple;

// PID Controller
double Kp = 850, Ki = 0.5, Kd = 0.1; // Just some initial deault tuning values. These will be overridden by the autotuner.
double currentTemp, pidOutput, targetTemp;
PID myPID(&currentTemp, &pidOutput, &targetTemp, Kp, Ki, Kd, DIRECT);
int pidControlRange = 10; // Only use the PID Controller when temp is within range

// PID Time Proportional Output
int windowSize = 1000; // 1 second TPO window
unsigned long windowStartTime;

// PID Autotuner
PID_ATune aTune(&currentTemp, &pidOutput);
bool                tuning = false;
double           aTuneStep = 500;
double          aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

// Serial Output
String logLines[2] = {"Start", "End"};

//  =========
//  = Setup =
//  =========
void setup() {
  Serial.begin(9600);

  // Splash Screen
  printLog("SousVido v0.01", "by Jevon Wild");

  // Setup Pins
  pinMode(HeaterLEDPin,      OUTPUT);
  pinMode(HeaterPin,         OUTPUT);
  pinMode(ThermocouplePin,   INPUT);
  pinMode(ButtonPin,         INPUT);
  digitalWrite(HeaterLEDPin, LOW);
  digitalWrite(HeaterPin,    LOW);

  // Initialize the thermocouple
  sensors.begin();
  if (!sensors.getAddress(waterThermocouple, 0)) {
    // If there's no Thermocouple found, do not continue.
    Serial.println("Error: Thermocouple Not Found.");
    // while(!sensors.getAddress(waterThermocouple, 0)) {
    //   delay(1000);
    // }
  }
  sensors.setResolution(waterThermocouple, 12); // Maximum/slowest resolution is 12. Minimum/fastest is 9.
  sensors.setWaitForConversion(false); // Sets up the thermocouple to be asynchronous;
  sensors.requestTemperatures(); // start an initial async temp reading

  // Set up a Timer to run driveOutput() periodically. This is the driver for
  // the Time Proportional Output, engaging/disengaging the Heaters based on
  // the output from the PID Controller while in the RUNNING state.
  MsTimer2::set(10, driveOutput); // 10ms period
  MsTimer2::start();

  delay(1000);
  // Pause notification.
  printLog("Unpause when", "ready.");
  delay(1000);
}



//  ========
//  = Loop =
//  ========
void loop() {
  handleButtonState(); // Change state based on Pause Button input

  syncTemps(); // keep our current/target temps accurate regardless of state

  // Change state if temperature is out of PID range
  if(currState != PAUSED){
    float error = targetTemp - currentTemp;
    if(error > pidControlRange){
      changeState(PRIMING);
    } else if(error < -pidControlRange) {
      changeState(RESTING);
    } else if (currState != RUNNING && currState != TUNING) {
      // We're within PID Control Range – time for a tune!
      changeState(TUNING);
    }
  }


  switch (currState) {
    case PAUSED:
      pause();
      break;
    case PRIMING:
      prime();
      break;
    case RESTING:
      rest();
      break;
    case TUNING:
      tune();
      break;
    case RUNNING:
      run();
      break;
  }
}



//  ============================================================================
//  = driveOutput                                                              =
//  = -----------                                                              =
//  = This is the driver for the Time Proportional Output, engaging or         =
//  = disengaging the Heaters based on the output from the PIDController while =
//  = in the RUNNING state.                                                    =
//  ============================================================================
void driveOutput() {
  long now = millis();

  // "on time" is proportional to the PID output
  if(currState == RUNNING || currState == TUNING) {
    if((now - windowStartTime) > windowSize) {
      windowStartTime += windowSize;
    }
    if((pidOutput > 100) && (pidOutput > (now - windowStartTime))) {
      engageHeaters();
    } else {
      disengageHeaters();
    }
  }
}


//  ============================================================================
//  = printLog                                                                 =
//  = --------                                                                 =
//  = Compares incoming log lines with the previous ones, and prints them out  =
//  = when they've changed.                                                    =
//  = Note: This is temporary, to be replaced with an LCD output.              =
//  ============================================================================
void printLog(String lineOne, String lineTwo) {
  if(logLines[0] != lineOne || logLines[1] != lineTwo){
    logLines[0] = lineOne;
    logLines[1] = lineTwo;
    Serial.println("================");
    Serial.println(logLines[0]);
    Serial.println(logLines[1]);
  }
}


//  ============================================================================
//  = Pause                                                                    =
//  = -----                                                                    =
//  = The State handler for "PAUSED"                                           =
//  ============================================================================
void pause() {
  myPID.SetMode(MANUAL);
  disengageHeaters();
}



//  ============================================================================
//  = Prime                                                                    =
//  = -----                                                                    =
//  = The State handler for "PRIMING"                                          =
//  ============================================================================
void prime() {
  printTemps();
  myPID.SetMode(MANUAL);
  engageHeaters();
}



//  ============================================================================
//  = Rest                                                                     =
//  = ----                                                                     =
//  = The State handler for "RESTING"                                          =
//  ============================================================================
void rest() {
  printTemps();
  myPID.SetMode(MANUAL);
  disengageHeaters();

  float error = targetTemp - currentTemp;
  if(error > pidControlRange) {
    changeState(PRIMING);
  } else if(error > -10) {
    changeState(TUNING);
  }
}



//  ============================================================================
//  = Tune                                                                     =
//  = ----                                                                     =
//  = The State handler for "TUNING"                                           =
//  ============================================================================
void tune() {
  printTemps();
  myPID.SetMode(AUTOMATIC);

  if(tuning && aTune.Runtime()){
    // Tuning was started, and now it's done.
    tuning = false;

    // Extract the auto-tune calculated parameters
    Kp = aTune.GetKp();
    Ki = aTune.GetKi();
    Kd = aTune.GetKd();

    changeState(RUNNING);
  } else if(abs(targetTemp - currentTemp) < 0.5){
    tuning = true;
    // Tuning hasn't been started yet, but we're within range.
    // set up the auto-tune parameters
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    aTune.SetControlType(1); // Set the control type to PID (default is 0, PI)
  }

  myPID.SetTunings(Kp,Ki,Kd);
}



//  ============================================================================
//  = Run                                                                      =
//  = ---                                                                      =
//  = The State handler for "RUNNING"                                          =
//  ============================================================================
void run() {
  printTemps();
  // Setup the PID Controller if it isn't already running
  if(myPID.GetMode() != AUTOMATIC){
    myPID.SetMode(AUTOMATIC);
    myPID.SetTunings(Kp,Ki,Kd);
    windowStartTime = millis();
  }

  myPID.Compute(); // Compute PID Output, which will be used driveOutput is run.
}



//  ============================================================================
//  = disengageHeaters                                                         =
//  = ----------------                                                         =
//  = Turns off the heaters via HeaterPin                                      =
//  ============================================================================
void disengageHeaters() {
  digitalWrite(HeaterLEDPin, LOW);
  digitalWrite(HeaterPin, LOW);
}



//  ============================================================================
//  = engageHeaters                                                            =
//  = -------------                                                            =
//  = Turns on the heaters via HeaterPin                                       =
//  ============================================================================
void engageHeaters() {
  digitalWrite(HeaterLEDPin, HIGH);
  digitalWrite(HeaterPin, HIGH);
}



//  ============================================================================
//  = printTemps                                                               =
//  = ----------                                                               =
//  = Prints current and target temperatures                                   =
//  ============================================================================
void printTemps() {
  String displayCurrentTemp = tempString(currentTemp, displayFahrenheit);
  String displayTargetTemp = tempString(targetTemp, displayFahrenheit);

  String stateNames[5] = { "PAUSED", "PRIMING", "RESTING", "TUNING", "RUNNING" };
  String lineOne = "State: ";
  lineOne += stateNames[currState];
  String lineTwo;
  lineTwo += "cur: ";
  lineTwo += displayCurrentTemp;
  lineTwo += "  tar: ";
  lineTwo += displayTargetTemp;

  printLog(lineOne, lineTwo);

  // Serial.println("Current: " + displayCurrentTemp);
  // Serial.println(" Target: " + displayTargetTemp);
}



//  ============================================================================
//  = changeState                                                              =
//  = -----------                                                              =
//  = Changes current state to the input newState                              =
//  ============================================================================
void changeState(State newState) {
  if(currState != newState){
    prevState = currState;
    currState = newState;
    stateChangedAt = millis();
  }
}



//  ============================================================================
//  = handleButtonState                                                        =
//  = -----------------                                                        =
//  = Reads the button state, and toggles current state as necessary           =
//  ============================================================================
void handleButtonState() {
  if(digitalRead(ButtonPin) == HIGH && millis() - stateChangedAt > buttonBuffer){
    changeState(currState == PAUSED ? prevState : PAUSED);
  }
}


//  ============================================================================
//  = tempString                                                               =
//  = ----------                                                               =
//  = Returns a string representation of a float temperature, converting to    =
//  = fahrenheit as necessary                                                  =
//  ============================================================================
String tempString(float temperature, bool convertToFahrenheit) {
  String retval;
  int displayTemp = (int)round(convertToFahrenheit ? celsiusToFahrenheit(temperature) : temperature);
  retval += displayTemp;
  retval += convertToFahrenheit ? "F" : "C";
  return retval;
}



//  ============================================================================
//  = celsiusToFahrenheit                                                      =
//  = -------------------                                                      =
//  = Returns the Fahrenheit equivalent of the input Celsius temperature       =
//  ============================================================================
float celsiusToFahrenheit(float celsius) {
  return celsius * (9.0/5.0) + 32.0;
}



//  ============================================================================
//  = syncTemps                                                                =
//  = ---------                                                                =
//  = Grab the current and target temps, and assign them to the appriopriate   =
//  = global variables                                                         =
//  ============================================================================
void syncTemps() {
  if(sensors.isConversionAvailable(0)) {
    // If there is a temperature reading available, grab it and start another async reading.
    currentTemp = sensors.getTempC(waterThermocouple);
    sensors.requestTemperatures();
  }
  targetTemp = gettargetTemp();
}



//  ============================================================================
//  = gettargetTemp                                                            =
//  = -------------                                                            =
//  = Returns the target temperature from a 10k potentiometer in degrees C     =
//  ============================================================================
float gettargetTemp() {
  // Read the current potentiometer position in 0-1023
  int potPosition = analogRead(PotentiometerPin);

  // convert 0-1023 to 0 - 1
  float normalizedpotPosition = (float)potPosition / 1023.0;

  // Convert 0-1 to 48.9-100 (Celsius)
  float targetTemperature = normalizedpotPosition * 51.1 + 48.9;

  return targetTemperature;
}

