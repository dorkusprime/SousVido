/*
  Sousvido
  Jevon Wild < github (at) jevonwild (dot) com >

  It's a sous vide cooker, in Arduino! Some parts based heavily on Adafruit's
  Sous Viduino <https://github.com/adafruit/Sous_Viduino>.

  Uses a potentiometer for setting temperature, and relies entirely on the
  autotune library to tune the PID controller. This is for applications without
  a consistent container or heating elements.

  This implementation also uses a Piezo Speaker when switching in- and out- of
  RUNNING state, along with an LED indicator
*/

// LiquidCrystal LCD Library
#include <LiquidCrystal.h>

// Libraries for the PID control system and autotune
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the digital thermocouple
#include <OneWire.h>
#include <DallasTemperature.h>

// WAY more readable code than using the Timer/Counter Control Register directly
#include <MsTimer2.h>

// Simplify pin interrupt code similar to MsTimer2
#include <EnableInterrupt.h>

// Math is FUN! But srsly, let's use a library to handle it.
#include <math.h>



//  ===================
//  = Pin Definitions =
//  ===================
#define PotentiometerPin A0
#define  ThermocouplePin 8
#define   ReadyBuzzerPin 9
#define      ReadyLEDPin 10
#define        HeaterPin 11
#define        ButtonPin 12
#define          PumpPin 13

// LCD Bus Pins
#define           LCD_RS 2
#define           LCD_EN 3
#define          LCD_DB4 4
#define          LCD_DB5 5
#define          LCD_DB6 6
#define          LCD_DB7 7



//  ==========================
//  = Buzzer Tone Deinitions =
//  ==========================
#define  tone_c     3830    // 261 Hz
#define  tone_d     3400    // 294 Hz
#define  tone_e     3038    // 329 Hz
#define  tone_f     2864    // 349 Hz
#define  tone_g     2550    // 392 Hz
#define  tone_a     2272    // 440 Hz
#define  tone_b     2028    // 493 Hz
#define  tone_C     1912    // 523 Hz



//  ===========================
//  = Settings & Initializers =
//  ===========================
bool displayFahrenheit = true; // Set to true for F, false for C

// Pause Button buffer
int buttonBuffer = 400; // tweak this if the pause button is acting up

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

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

// Thermocouple
OneWire oneWire(ThermocouplePin);
DallasTemperature sensors(&oneWire);
DeviceAddress thermocouple;

// PID Controller
double defaultKp = 850;
double defaultKi = 0.5;
double defaultKd = 0.1;
double currentTemp, pidOutput, targetTemp;
PID myPID(&currentTemp, &pidOutput, &targetTemp, defaultKp, defaultKi, defaultKd, DIRECT);
int pidDeactivationRange = 14; // Deactivate the PID Controller when temp falls out of range. Note: CELCIUS!
int pidActivationRange = 1; // Only activate the PID Controller when temp is within range. Note: CELCIUS!

// PID Time Proportional Output
int windowSize = 10000; // 10 second TPO window
unsigned long windowStartTime;

// PID Autotuner
PID_ATune aTune(&currentTemp, &pidOutput);
bool               aTuning = false;
double           aTuneStep = 500;
double          aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

// LCD Output
String logLines[2] = {"Start", "End"};



//  =========
//  = Setup =
//  =========
void setup() {
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Splash Screen
  printLCD("SousVido v0.50", "by Jevon Wild");

  // Setup Pins
  pinMode(HeaterPin,        OUTPUT);
  pinMode(ReadyLEDPin,      OUTPUT);
  pinMode(ReadyBuzzerPin,   OUTPUT);

  pinMode(ButtonPin,        INPUT);
  pinMode(ThermocouplePin,  INPUT);

  digitalWrite(HeaterPin,   LOW);

  // Initialize the thermocouple
  sensors.begin();
  if (!sensors.getAddress(thermocouple, 0)) {
    // If there's no Thermocouple found, do not continue.
    Serial.println("Error: Thermocouple Not Found.");
    Serial.println(sensors.getAddress(thermocouple, 0));
    while(!sensors.getAddress(thermocouple, 0)) {
      delay(1000);
    }
  }
  sensors.setResolution(thermocouple, 12); // Maximum/slowest resolution is 12. Minimum/fastest is 9.
  sensors.setWaitForConversion(false); // Sets up the thermocouple to be asynchronous;
  sensors.requestTemperatures(); // start an initial async temp reading

  // Setup the PID
  myPID.SetSampleTime(1000); // How often the PID is evaluated
  myPID.SetOutputLimits(500, windowSize);

  // Set up a Timer to run driveOutputAsync() periodically. This is the driver for
  // the Time Proportional Output, engaging/disengaging the Heaters based on
  // the output from the PID Controller while in the RUNNING state.
  MsTimer2::set(10, driveOutputAsync); // 10ms period
  MsTimer2::start();

  // Setup an interrupt on the Ready LED, for playing a tone when it changes
  enableInterrupt(ReadyLEDPin, playReadyBuzzer, CHANGE);

  delay(3000);
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
    if(error > pidDeactivationRange){
      changeState(PRIMING);
    } else if(error < -pidDeactivationRange) {
      changeState(RESTING);
    } else if (currState != RUNNING && abs(error) < (pidActivationRange)) {
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
//  = Pause                                                                    =
//  = -----                                                                    =
//  = The State handler for "PAUSED"                                           =
//  ============================================================================
void pause() {
  digitalWrite(ReadyLEDPin, LOW);
  myPID.SetMode(MANUAL);
  disengageHeaters();
  printLCD(" --- PAUSED --- ", "");
}



//  ============================================================================
//  = Prime                                                                    =
//  = -----                                                                    =
//  = The State handler for "PRIMING"                                          =
//  ============================================================================
void prime() {
  printTemps();
  digitalWrite(ReadyLEDPin, LOW);
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
  digitalWrite(ReadyLEDPin, LOW);
  myPID.SetMode(MANUAL);
  disengageHeaters();
}



//  ============================================================================
//  = Tune                                                                     =
//  = ----                                                                     =
//  = The State handler for "TUNING"                                           =
//  ============================================================================
void tune() {
  printTemps();
  digitalWrite(ReadyLEDPin, LOW);
  if(myPID.GetMode() != AUTOMATIC) {
    // Fire up the PID!
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
  }

  if(aTuning && aTune.Runtime()){
    // Tuning was started, and now it's done.
    Serial.println("Autotune complete.");
    aTuning = false;

    // Set the K parameters to our newly autotuned ones
    myPID.SetTunings(aTune.GetKp(), aTune.GetKi(), aTune.GetKd());

    changeState(RUNNING);
  } else if(!aTuning && abs(targetTemp - currentTemp) < 0.5){
    // Autotuning hasn't been started yet, but we're within range. Start it!
    Serial.println("Starting autotune.");
    aTuning = true;
    windowStartTime = millis();

    // If we're TUNING, make sure we start off with default K parameters.
    myPID.SetTunings(defaultKp, defaultKi, defaultKd);

    // set up the auto-tune parameters
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    aTune.SetControlType(1); // Set the control type to PID (default is 0, PI)
  }

  myPID.Compute(); // Compute PID Output, which will be used when driveOutputAsync is run.
}



//  ============================================================================
//  = Run                                                                      =
//  = ---                                                                      =
//  = The State handler for "RUNNING"                                          =
//  ============================================================================
void run() {
  printTemps();
  digitalWrite(ReadyLEDPin, HIGH);

  myPID.Compute(); // Compute PID Output, which will be used when driveOutputAsync is run.
}



//  ============================================================================
//  = driveOutputAsync                                                         =
//  = ----------------                                                         =
//  = This is the driver for the Time Proportional Output, engaging or         =
//  = disengaging the Heaters based on the output from the PIDController while =
//  = in the RUNNING state.                                                    =
//  ============================================================================
void driveOutputAsync() {
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
//  = printLCD                                                                 =
//  = --------                                                                 =
//  = Compares incoming log lines with the previous ones, and prints them out  =
//  = when they've changed.                                                    =
//  = Note: This is temporary, to be replaced with an LCD output.              =
//  ============================================================================
void printLCD(String lineOne, String lineTwo) {
  if(logLines[0] != lineOne || logLines[1] != lineTwo){
    logLines[0] = lineOne;
    logLines[1] = lineTwo;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(lineOne);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(lineTwo);
    lcd.print("                ");

  }
}



//  ============================================================================
//  = disengageHeaters                                                         =
//  = ----------------                                                         =
//  = Turns off the heaters via HeaterPin                                      =
//  ============================================================================
void disengageHeaters() {
  digitalWrite(HeaterPin, LOW);
}



//  ============================================================================
//  = engageHeaters                                                            =
//  = -------------                                                            =
//  = Turns on the heaters via HeaterPin                                       =
//  ============================================================================
void engageHeaters() {
  digitalWrite(HeaterPin, HIGH);
}



//  ============================================================================
//  = playReadyBuzzer                                                          =
//  = ---------------                                                          =
//  = Plays the buzzer                                                         =
//  ============================================================================
void playReadyBuzzer() {
  if (digitalRead(ReadyLEDPin) == HIGH) {
    playUp();
  } else {
    playDown();
  }
}



//  ============================================================================
//  = playUp                                                                   =
//  = ------                                                                   =
//  = Plays an upward melody                                                   =
//  ============================================================================
void playUp() {
  int melody[] = {  tone_c,  tone_d,  tone_g,  tone_C};
  int beats[]  = { 24, 24, 24, 64};
  int count = sizeof(melody) / 2; // Melody length, for looping.

  playMelody(melody, beats, count);
}



//  ============================================================================
//  = playDown                                                                 =
//  = --------                                                                 =
//  = Plays a downward melody                                                  =
//  ============================================================================
void playDown() {
  int melody[] = {  tone_C,  tone_g,  tone_d,  tone_c};
  int beats[]  = { 24, 24, 24, 64};
  int count = sizeof(melody) / 2; // Melody length, for looping.

  playMelody(melody, beats, count);
}



//  ============================================================================
//  = playMelody                                                               =
//  = ----------                                                               =
//  = Plays any input melody                                                   =
//  ============================================================================
void playMelody(int melody[], int beats[], int notes) {
  for (int i=0; i<notes; i++) {
    playTone(melody[i], (beats[i] * long(4000)));
  }
}



//  ============================================================================
//  = playTone                                                                 =
//  = --------                                                                 =
//  = Plays any input tone                                                     =
//  ============================================================================
void playTone(int tone, long duration) {
  long elapsed_time = 0;
  while (elapsed_time < duration) {
    // Up
    digitalWrite(ReadyBuzzerPin,HIGH);
    delayMicroseconds(tone / 2);

    // Down
    digitalWrite(ReadyBuzzerPin, LOW);
    delayMicroseconds(tone / 2);

    // Keep track of how long we pulsed
    elapsed_time += (tone);
  }
}



//  ============================================================================
//  = printTemps                                                               =
//  = ----------                                                               =
//  = Prints current and target temperatures                                   =
//  ============================================================================
void printTemps() {
  String displayCurrentTemp = tempString(currentTemp, displayFahrenheit);
  String displayTargetTemp = tempString(targetTemp, displayFahrenheit);

  String lineOne = "    Temp: ";
  lineOne += displayCurrentTemp;

  String lineTwo = "  Target: ";
  lineTwo += displayTargetTemp;

  printLCD(lineOne, lineTwo);
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
    double response = sensors.getTempC(thermocouple);
    if(response > 0){
      // allow for thermocouple weirdness
      currentTemp = (currentTemp + response) / 2;
    }
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
  // Take 10 readings, to smooth out any potentiometer bumps
  int samplesTotal = 0;
  for (int i=0; i< 10 ; i++) {
    samplesTotal += analogRead(PotentiometerPin);
  }

  // The average potentiometer position in 0-1023
  float potPosition = (float)samplesTotal/10.0;

  // convert 0-1023 to 0 - 1
  float normalizedpotPosition = potPosition / 1023.0;

  // Convert 0-1 to 48.9-100 (Celsius)
  float targetTemperature = normalizedpotPosition * 51.1 + 48.9;

  return targetTemperature;
}

