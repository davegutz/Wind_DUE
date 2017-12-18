//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#ifdef ARDUINO
#include <Servo.h>
#include <Arduino.h>     // Used instead of Print.h - breaks Serial
  #ifdef __AVR_ATmega328P__
  #define UNO
  #endif
  #ifdef __AVR_ATmega2560__
  #define MEGA
  #endif
  #ifdef __SAM3X8E__
  #define DUE
  #endif
#else                    // Photon
#define PHOTON
#include "application.h" // Should not be needed if file ino or Arduino
#ifdef PARTICLE
#include <Particle.h>     // Needed for CLI standalone
#endif
SYSTEM_THREAD(ENABLED); // Make sure code always run regardless of network status
#endif


#include "myAnalyzer.h"
#include "myTables.h"
#include "math.h"

// Test features
// in myClaw.h  #define KIT   1   // -1=Photon, 0-4 = Arduino
int     ttype = 0;       // 0=STEP, 1=FREQ, 2=VECT, 3=RAMP (ramp is open loop only), 4=RAND, 5=SQUARE (at step size, 5 sec period)
//#define CALIBRATING    // Use this to port converted v4 to the vpot serial signal for calibration
int     verbose = 0;     // [0] Debug, as much as you can tolerate.   For talk() set using "v#"
bool    bare = false;    // [false] The microprocessor is completely disconnected.  Fake inputs and sensors for test purposes.  For talk() set using "b"
bool    potOverride = false; // [false] The pot is over-ridden by talk()
bool    dry = false;     // [false] The turbine and ESC are disconnected.  Fake inputs and sensors for test purposes.  For talk() set using "t"
double  stepVal = 6;     // [6] Step input, %nf.  Try to make same as freqRespAdder
bool    plotting = true; // [false] This is for Serial Plotter compatible output (menu - Tools - Serial Plotter)


/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

Connections for Photon:
  ESC ------------------- Photon
    BLK ------------------- GND
    WHT ------------------- PWM Output (A4)
  ESC----------------- 3-wire DC Servomotor
    Any three to any three
  DPST switch-------------Photon
    GND-------------------D4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Photon
      R---------------------D2
    Push button
      HI--------------------3.3V
      LO--------------------10K to GND
  F2V---------------------Photon
    V5/V10----------------Analog In A2
    GND-------------------GND
  POT---------------------Photon
    VHI ------------------3.3v
    VLO ------------------GND
    WIPE -----------------Analog In A0
  LED to indicate frequency response----Digital Output (D7)
  Hardware Platform:
    Microcontroller:  Particle Photon
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)


Connections for Arduino:
  POW ------------------- Arduino
    POS/PLUS -------------- Digital out D3
    other ----------------- GND
  ESC ------------------- Arduino
    BLK ------------------- GND
    WHT ------------------- PWM Output (5)
    RED ------------------- Digital in D6
  ESC----------------- 3-wire DC Servomotor
    Any three to any three
  F2V-----------------Arduino
    V5/V10----------------Analog In A2
    GND-------------------GND
  DPST switch-------------Arduino
    CTR-------------------4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Arduino
    R---------------------2
  Push button
    HI--------------------3.3V
    LO--------------------10K to GND
  POT---------------------Arduino
    VHI ------------------5V
    VLO ------------------GND
    WIPE -----------------Analog In A0
  BUTTON ----------------Ardunio  D2
    see https://www.arduino.cc/en/Tutorial/Button
  LED to indicate frequency response----Digital Output (7)
  JUMPER---------------4 to GND for Closed Loop
  Hardware Platform:
    Microcontroller:  Arduino Uno R3
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

  Reaquirements:
  Prime:
  1.  Inputs:  fan speed, pushbutton, slide switch, potentiometer
  2.  Outputs: ESC command using TTL/PWM servo control library
  3.  Manually sweep ESC command from min to max using pot.
  4.  Limit ESC command for safety using configurable parameters.
  5.  Turn on to minimum ESC input level.  Calibration of the ESC is done elsewhere.
  6.  Switch from open to closed loop and vice versa with small, <5 deg throttle
      bump, in response to switch change.
  7.  Perform frequency response analysis in response to button push.
  8.  Update closed loop algorithms, sense inputs, and produce outputs at rate
      at least as fast as 0.015 second update rate.
  Secondary:
  1.  Repeated Pushbutton toggles frequency response in progress.   When
      restarting, it begins completely fresh.
  2.  For non-Arduino, may also use a software Pushbutton - send string on serial.
  3.  Embed a plant model for dry code checkouts.
  4.  Filter pot noise.

  Tasks TODO:
  1.  Photon frequency response with button

  Revision history:
    31-Aug-2016   DA Gutz   Created
    13-Sep-2016   DA Gutz   Initial analyzing
    30-Sep-2016   DA Gutz   Arduino added
    10-Oct-2016   DA Gutz   First frequency response completion
    30-Oct-2016   DA Gutz   Gain scheduling
    16-Nov-2016   DA Gutz   Retune again again

  Distributed as-is; no warranty is given.
*/

// Test features usually commented
//

// Disable flags if needed.  Usually commented
//#define DISTURB_CONTROL                       // Use disturbance rejection gains in CLAW

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
#define POWER_IN_PIN 6                                          // Read level of power on/off (D6)
#define POWER_EN_PIN 3                                          // Write power enable discrete (D3)
#define BUTTON_PIN 2                                           // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN 5                                              // PWM output (PD5)
#define POT_PIN A0                                             // Potentiometer input pin on Arduino (PC0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Arduino (PC2)
#define CL_PIN 4                                               // Closed loop 3-way switch 5V or GND (D4 to GND)
#define CLOCK_TCK 16UL                                         // Clock tick resolution, micros
#define INSCALE 1023.0                                         // Input full range from OS
const double vpotHalfDB = 0.0;                                 // Half deadband sliding deadband filter, volts
const double POT_MAX = 3.3;                                    // Maximum POT value, vdc
const double F2V_MAX = 3.3;                                    // Maximum F2V value, vdc
const double POT_BIA = 0.10 + vpotHalfDB;                      // Pot adder, vdc.   0.1 is observed vpot+  min with 3.3/1023
const double POT_SCL = (3.1 - vpotHalfDB - POT_BIA) / POT_MAX; // Pot scalar, vdc.   3.1 is observed vpot- max with 3.3/1023
#else                                                          // Photon
#define BUTTON_PIN D2                                          // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN A4                                             // PWM output (A4)
#define POT_PIN A0                                             // Potentiometer input pin on Photon (A0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Photon (A2)
#define CL_PIN D0                                              // Closed loop 3-way switch 3.3V or GND  (D0)
#define CLOCK_TCK 8UL                                          // Clock tick resolution, micros
#define INSCALE 1023.0                                         // Input full range from OS
const double vpotHalfDB = 0.0;                    // Half deadband sliding deadband filter, volts
const double POT_MAX = 3.3;                       // Maximum POT value, vdc
const double F2V_MAX = 3.45;                      // Maximum F2V value, vdc
const double POT_BIA = 0.0;                       // Pot adder, vdc
const double POT_SCL = (3.3 - POT_BIA) / POT_MAX; // Pot scalar, vdc
#endif
//********constants for all*******************
#ifdef CALIBRATING
#define PUBLISH_DELAY 150000UL      // Time between cloud updates (), micros
#else
#define PUBLISH_DELAY 15000UL       // Time between cloud updates (), micros
#endif
#define CONTROL_DELAY    15000UL    // Control law wait (), micros
#define CONTROL_10_DELAY  150000UL   // Control law wait (), micros
#define CONTROL_100_DELAY 1500000UL  // Control law wait (), micros
#define FR_DELAY 4000000UL    // Time to start FR, micros
const double F2V_MIN = 0.0;   // Minimum F2V value, vdc
const double POT_MIN = 0;     // Minimum POT value, vdc
const double DENS_SI = 1.225; // Air density, kg/m^3

// Test
testType testOnButton = STEP;
bool freqResp = false;             // Perform frequency response test status
//bool vectoring = false;            // Perform vector test status
const int nsigFn = 4;              // Length of fn
const int ntfFn = 2;               // Number of transfer  functions to calculate <= length(ix)
double fn[4] = {0, 0, 0, 0};       // Functions to analyze
const int ix[2] = {0, 0};          // Indeces of fn to excitations
const int iy[2] = {1, 2};          // Indeces of fn to responses
const double freqRespScalar = 1e8; // Use 40 for +/-3 deg, 20 for +/-6 deg, 13 for +/-10 at 50% Nf
const double freqRespAdder = 6;    // +/- deg

//
// Dependent includes
#include "myCLAW.h"
#include "myFilters.h"

// Global variables
double throttle = -5; // Servo value, 0-179 degrees
bool powerEnable = false; // Turn on ESC power
char buffer[256];
LagTustin *throttleFilter; // Tustin lag noise filter
FRAnalyzer *analyzer;      // Frequency response analyzer
Servo myservo;             // create servo object to control dc motor
ControlLaw *CLAW;          // Control Law
Debounce *ClPinDebounce;  // Input switch status
Debounce *PowerDebounce;  // Power status
Debounce *ButtonDebounce; // Pushbutton status
TFDelay  *EnableDelayed;  // Power wait for Serial turn on
TFDelay  *PowerDelayed;   // ESC wait for boot
String inputString = "";        // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete

// Test vector setup (functions at bottom of this file)
bool Vcomplete(void);
double Vcalculate(double);
void Vcomplete(bool);
const double Vtv_[] =  {0,  8,  16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104,  112,  120,  128,  136, 144, 152, 156}; // Time, s
const double Vvv_[] =  {10, 20, 25, 30, 42, 48, 56, 62, 77, 90, 96, 90, 77, 62,   56,   48,   42,   30,  25,  20,  10};  // Excitation
const unsigned int Vnv_ = sizeof(Vtv_)/sizeof(double);  // Length of vector
double Voutput_ = 0;        // Excitation value
double Vtime_ = 0;          // Time into vector, s
double VtnowStart_ = 0;     // now time of vector start reference, s
bool Vcomplete_ = false;    // Status of vector, T=underway
unsigned int Viv_ = 0;      // Index of present time in vector


// Ramp vector setup (functions at bottom of this file)
bool Rcomplete(void);
double Rcalculate(double);
void Rcomplete(bool);
const double Rtv_[] =  {0,  8,  68, 78, 138, 148}; // Time, s
const double Rvv_[] =  {10, 10, 96, 96, 10,  10};  // Excitation
const unsigned int Rnv_ = sizeof(Rtv_)/sizeof(double);  // Length of vector
double Routput_ = 0;        // Excitation value
double Rtime_ = 0;          // Time into vector, s
double RtnowStart_ = 0;     // now time of vector start reference, s
bool Rcomplete_ = false;    // Status of vector, T=underway
unsigned int Riv_ = 0;      // Index of present time in vector

// Test vector setup (functions at bottom of this file)
bool RandComplete(void);
double RandCalculate(double);
void RandComplete(bool);

// Calibration
bool Calibrate(void);
void talk(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,
    int *buttonState, ControlLaw *CLAW, const int potThrottle);

void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
#endif
  pinMode(POWER_IN_PIN, INPUT);
  pinMode(POWER_EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(115200);
  SerialUSB.begin(115200);
  myservo.attach(PWM_PIN, 1000, 2000); // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(F2V_PIN, INPUT);
  pinMode(CL_PIN, INPUT);

  // Lag filter
  double T = float(CONTROL_DELAY) / 1000000.0;
  throttleFilter = new LagTustin(T, tau, -0.1, 0.1);

  // Frequency Response
  //                        wmin    wmax  dw    minCy numCySc  iniCy  wSlow
  analyzer = new FRAnalyzer(-0.8,   1.4,  0.1,  2,    1.0,     6,     1 / tauG,
                            double(CONTROL_DELAY / 1e6), ix, iy, nsigFn, ntfFn, "t,ref,exc,thr,mod,nf,T"); // 15 ms any

  myservo.write(180);
  digitalWrite(POWER_EN_PIN, LOW);

  
// Serial headers used by plotting programs
// Header for analyzer.  TODO:  should be done in analyzer.cpp so done when needed
#ifndef ARDUINO
  delay(5000); // Allow time to start serial monitor for Photon.  Ardino auto-starts
#endif
  /*
    analyzer->publish();
    Serial.println("");
    */
  // Header for time data
  if (verbose > 0)
  {
    //******************************************************************************************************************************
#ifdef CALIBRATING
    sprintf(buffer, "\ntime,mode,vf2v,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
#else
    sprintf(buffer, "\ntime,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
#endif
    Serial.print(buffer);
  }

  // Serial Event to allow switches to be passed in by user Serial transmit
  inputString.reserve(200); // Reserve 200 bytes for inputString Serial Event

  // Instatiate gain scheduling tables
  CLAW = new ControlLaw(T, DENS_SI);
  ClPinDebounce = new Debounce((digitalRead(CL_PIN) == HIGH), 3);
  PowerDebounce = new Debounce(false, 1);
  ButtonDebounce = new Debounce(0, 3);
  PowerDelayed  = new TFDelay(false, 2.0, 0.0, T*100);
  EnableDelayed = new TFDelay(false, 5.0, 0.0, T*100);

#ifdef ARDUINO
  delay(100);
#else
  delay(1000);
  WiFi.off();
  delay(1000);
#endif
}

void loop()
{
  static bool vectoring = false;          // Perform vector test status
  static bool powered = false;            // Monitor ESC power
  static bool powerToCal = false;         // Wait for ESC bootup
  int buttonState = 0;                    // Pushbutton
  static bool closingLoop = false;        // Persisted closing loop by pin cmd, T/F
  static bool stepping = false;           // Step by Photon send String
  bool control;                           // Control frame, T/F
  bool control10;                         // Control 10T frame, T/F
  bool control100;                        // Control 100T frame, T/F
  bool publish;                           // Publish, T/F
  static bool analyzing;                         // Begin analyzing, T/F
  unsigned long now = micros();           // Keep track of time
  static unsigned long start = 0UL;       // Time to start looping, micros
  double elapsedTime;                     // elapsed time, micros
  static double updateTime = 0.0;         // Control law update time, sec
  static unsigned long lastControl = 0UL; // Last control law time, micros
  static unsigned long lastPublish = 0UL; // Last publish time, micros
  static unsigned long lastControl10 = 0UL;    // Last control 10T time, micros
  static unsigned long lastControl100 = 0UL;    // Last control 100T time, micros
#ifdef ARDUINO
  static unsigned long lastButton = 0UL;  // Last button push time, micros
  static unsigned long lastFR = 0UL;      // Last analyzing, micros
#endif
  static int mode = 0;                    // Mode of operation First digit: closingLoop, Second digit: testOnButton, Third digit:  analyzing
  static int RESET = 1;                   // Dynamic reset
  const double RESEThold = 5;             // RESET hold, s
  static double exciter = 0;              // Frequency response excitation, fraction
                                          ////////////////////////////////////////////////////////////////////////////////////
  static double vf2v = 0;                 // Converted sensed back emf LM2907 circuit measure, volts
  static double vpot_filt = 0;            // Pot value, volts
  static double vpotDead = 0;             // Sliding deadband value, volts
  static double vpot = 0;                 // Pot value, volts
  static int f2vValue = INSCALE / 4;      // Dial raw value
  static int potValue = INSCALE / 3;      // Dial raw value
  static double potThrottle = 0;

  // Executive
  if (start == 0UL) start = now;
  elapsedTime = double(now - start) * 1e-6;
  publish = ((now - lastPublish) >= PUBLISH_DELAY - CLOCK_TCK / 2);
  if (publish)
  {
    lastPublish = now;
  }
  unsigned long deltaTick = now - lastControl;
  control = (deltaTick >= CONTROL_DELAY - CLOCK_TCK / 2);
  if (control)
  {
    updateTime = float(deltaTick) / 1000000.0;
    lastControl = now;
  }
  unsigned long deltaTick10 = now - lastControl10;
  control10 = (deltaTick10 >= CONTROL_10_DELAY - CLOCK_TCK / 2);
  if (control10)
  {
    lastControl10 = now;
  }
  unsigned long deltaTick100 = now - lastControl100;
  control100 = (deltaTick100 >= CONTROL_100_DELAY - CLOCK_TCK / 2);
  if (control100)
  {
    lastControl100 = now;
  }
  if (!bare)
  {
    if ( control )
    {
      closingLoop = ClPinDebounce->calculate(digitalRead(CL_PIN) == HIGH);
    }
    if ( control100 )
    {
      powered = PowerDebounce->calculate(digitalRead(POWER_IN_PIN) == HIGH);
      powerToCal = PowerDelayed->calculate(powered);
      powerEnable = EnableDelayed->calculate(true);
    }
    buttonState = ClPinDebounce->calculate(digitalRead(BUTTON_PIN));
  }
  if (buttonState == HIGH && (now - lastButton > 200000UL))
  {
    lastButton = now;
    if ( bare ) buttonState = LOW;  // Reset if bare
    switch ( testOnButton )
    {
      case FREQ:
      {
        analyzer->complete(freqResp); // reset if doing freqResp
        freqResp = !freqResp;
        break;
      }
      case STEP:
      {
        stepping  = true;
        stepVal  = -stepVal;
        break;
      }
      case VECT:
      {
        Vcomplete(vectoring); // reset if doing vector
        vectoring = !vectoring;
        break;
      }
      case RAMP:
      {
        Rcomplete(vectoring); // reset if doing vector
        vectoring = !vectoring;
        break;
      }
      case RAND:
      {
        RandComplete(vectoring); // reset if doing vector
        vectoring = !vectoring;
        break;
      }
    }
  }
  switch (ttype)
  {
  case (1):
    if ( freqResp) // FREQ
    analyzing = ( ((now - lastFR) >= FR_DELAY && !analyzer->complete()) );
    break;
  case (2):  // VECT
    if ( vectoring ) analyzing = !Vcomplete();
    break;
  case (3):   // RAMP
    if ( vectoring ) analyzing = !Rcomplete();
    break;
  case (4):  // RAND
    if ( control100 && vectoring  ) analyzing = !RandComplete();
    else analyzing = false;
    break;
  }
  mode = bare*10000 + closingLoop*1000 + dry*100 + testOnButton*10 + analyzing;

  // Discuss things with the user
// When open interactive serial monitor such as CoolTerm
// then can enter commands by sending strings.   End the strings with a real carriage return
// right in the "Send String" box then press "Send."
// String definitions are below.
  talk(&vectoring, &closingLoop, &stepping, &potValue, &buttonState, CLAW, potThrottle);

  // Interrogate analog inputs
  if (control)
  {
    if (!bare && !potOverride)
    {
      potValue = analogRead(POT_PIN);
      f2vValue = analogRead(F2V_PIN);
    }
    vf2v = double(f2vValue) / INSCALE * F2V_MAX;
    vpot = fmin(fmax((double(potValue) / INSCALE * POT_MAX - POT_BIA) / POT_SCL, POT_MIN), POT_MAX);
  }

  // Control law
  if (control)
  {
    vpotDead = fmax(fmin(vpotDead, vpot + vpotHalfDB), vpot - vpotHalfDB);
    if (!freqResp)
      vpot_filt = throttleFilter->calculate(vpotDead, RESET); // Freeze pot for FR
    potThrottle = vpot_filt * THTL_MAX / POT_MAX;      // deg
    double dNdT = P_LTALL_NG[1] / fmax(potThrottle, 1) / RPM_P;  // Rate normalizer, %Ng/deg
    potThrottle += stepping * stepVal / dNdT;

    throttle = CLAW->calculate(RESET, updateTime, closingLoop, analyzing, freqResp, vectoring, exciter, freqRespScalar, freqRespAdder, potThrottle, vf2v);
    if (elapsedTime > RESEThold)
      RESET = 0;
  }
  if ( control100)
  {
  }

  // Commands to Hardware
  if (control && !dry && !bare)
  {
    static bool calComplete = false;
    bool calibrate = powerToCal && potThrottle<=5 && !calComplete;
    if ( calibrate ) calComplete = Calibrate();
    else if ( calComplete )  myservo.write(throttle); 
    if ( powerEnable )  digitalWrite(POWER_EN_PIN, HIGH);
    else digitalWrite(POWER_EN_PIN, LOW);
  }

  // Calculate frequency response
  if (control)
  {
    fn[0] = throttle;
    fn[1] = CLAW->modelTS();
    fn[2] = CLAW->pcnt();
    fn[3] = CLAW->pcntRef();
    if (analyzing)
    {
      switch ( ttype )
      {
        case ( 1 ):  // FREQ
          if ( freqResp ) exciter = analyzer->calculate(fn, nsigFn); // use previous exciter for everything
          break;
        case ( 2 ):  // VECT
          if ( vectoring ) exciter = Vcalculate(elapsedTime);
          break;
        case ( 3 ):  // RAMP
          if ( vectoring ) exciter = Rcalculate(elapsedTime);
          break;
        case ( 4 ):  // RAND
          if ( control100 && vectoring ) exciter = RandCalculate(elapsedTime);
          break;
      }
    }
  }

  // Publish results to serial bus
  if (publish)
  {
    if ( plotting )
    {
      SerialUSB.print(CLAW->pcntRef());SerialUSB.print(",");
      SerialUSB.print(CLAW->pcnt());   SerialUSB.print(",");
      SerialUSB.print(CLAW->modelTS());SerialUSB.print(",");
      SerialUSB.print(CLAW->modelG()); SerialUSB.print(",");
      SerialUSB.print(throttle/1.8);   SerialUSB.print(",");
//      SerialUSB.print("100,-25,");
      SerialUSB.println("");
    }
    if (freqResp)
    {
      if (verbose > 1 || (testOnButton==STEP  && verbose>0) )
      {
        sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,",
                String(elapsedTime, 6).c_str(), String(CLAW->pcntRef()).c_str(),
                String(exciter).c_str(), String(throttle).c_str(),
                String(CLAW->modelTS()).c_str(), String(CLAW->pcnt()).c_str(),
                String(updateTime, 6).c_str());
        Serial.print(buffer);
        if (!analyzer->complete())
        {
          if ( ttype==1 ) analyzer->publish();
        }
        Serial.println("");
      }
    } // freqResp
    else
    {
      sprintf(buffer, "\ntime,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
      if (verbose > 0)
      {
        Serial.print(elapsedTime, 6);Serial.print(",");
        Serial.print(mode, DEC);Serial.print(", ");
        #ifdef CALIBRATING
          Serial.print(vf2v, 3);Serial.print(",");
        #else
          Serial.print(vpot, 3);Serial.print(",  ");
        #endif
        Serial.print(CLAW->pcntRef(), DEC);Serial.print(",");
        sprintf(buffer, "%s,", String(CLAW->pcnt()).c_str()); Serial.print(buffer);
        Serial.print(CLAW->modelTS(), DEC);Serial.print(",");
        Serial.print(CLAW->e(), DEC);Serial.print(",");
        Serial.print(CLAW->intState(), DEC);Serial.print(",");
        Serial.print(throttle, 0);Serial.print(",  ");
        Serial.print(CLAW->modelG(), DEC);Serial.print(",");
        Serial.print(updateTime, 6);Serial.println(",");
      }
    }
  } // publish
  switch ( ttype )
  {
    case (1):  // FREQ 
      if (analyzer->complete()) freqResp = false;
      break;
    case (2):  // VECT
      if (Vcomplete()) vectoring = false;
      break;
    case (3):  // RAMP
      if (Rcomplete()) vectoring = false;
      break;
    case (4):  // RAND
      if (control100 && RandComplete()) vectoring = false;
      break;
  }
}


/*
  Special handler that uses built-in callback.
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar=='\n' || inChar=='\0' || inChar==';' || inChar==',')
    {
      stringComplete = true;
     // Remove whitespace
      inputString.trim();
      inputString.replace(" ","");
      inputString.replace("=","");
      Serial.println(inputString);
    }
  }
}

// Vector calculator
double Vcalculate(const double tnow)
{
  if ( VtnowStart_ == 0 )
  {
    VtnowStart_ = tnow;  // First call sets time
    Vcomplete_ = false;
    Viv_ = 0;
  }
  // Find location in vector
  Vtime_ = tnow-VtnowStart_;
  while ( Vtv_[Viv_]<Vtime_ && Viv_<Vnv_ ) Viv_++;
  // Output
  if ( Viv_ == Vnv_ ) Vcomplete_ = true;
  unsigned int iv = Viv_;
  if ( iv==0 ) iv = 1;
  Voutput_ = Vvv_[iv-1];
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",iv=%s"), String(iv_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tv[iv]=%s"), String(tv_[iv_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Voutput_).c_str());        Serial.print(buffer);
*/
  return ( Voutput_ );
};

bool Vcomplete(void) { return (Vcomplete_); };

// Restart vector
void Vcomplete(const bool set)
{
  Viv_ = 0;
  Vcomplete_ = false;
  Vtime_ = 0;
  VtnowStart_ = 0;
};



// Ramp calculator
double Rcalculate(const double tnow)
{
  if ( RtnowStart_ == 0 )
  {
    RtnowStart_ = tnow;  // First call sets time
    Rcomplete_ = false;
    Riv_ = 0;
  }
  // Find location in vector
  Rtime_ = tnow-RtnowStart_;
  while ( Rtv_[Riv_]<Rtime_ && Riv_<Rnv_ ) Riv_++;   // iv is location past now
  // Output
  if ( Riv_ == Rnv_ ) Rcomplete_ = true;
  unsigned int ir = Riv_;
  if ( ir==0 ) ir = 1;
  Routput_ = (Rtime_-Rtv_[ir-1]) / (Rtv_[ir]-Rtv_[ir-1]) * (Rvv_[ir]-Rvv_[ir-1])  +  Rvv_[ir-1];
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",ir=%s"), String(ir_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tr[ir]=%s"), String(tr_[ir_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Routput_).c_str());        Serial.print(buffer);
*/
  return ( Routput_ );
};

bool Rcomplete(void) { return (Rcomplete_); };

// Restart ramp
void Rcomplete(const bool set)
{
  Riv_ = 0;
  Rcomplete_ = false;
  Rtime_ = 0;
  RtnowStart_ = 0;
};

// Random calculator
double RandCalculate(const double tnow)
{
  if ( VtnowStart_ == 0 )
  {
    VtnowStart_ = tnow;  // First call sets time
    Vcomplete_ = false;
  }
  // Find location in vector
  Vtime_ = tnow-VtnowStart_;
  Voutput_ = random(80);
  /*
          sprintf_P(buffer, PSTR("time=%s"), String(time_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",iv=%s"), String(iv_).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",tv[iv]=%s"), String(tv_[iv_]).c_str());        Serial.print(buffer);
          sprintf_P(buffer, PSTR(",output=%s\n"), String(Voutput_).c_str());        Serial.print(buffer);
*/
  return ( Voutput_ );
};

bool RandComplete(void) { return (Vcomplete_); };

// Restart vector
void RandComplete(const bool set)
{
  Vcomplete_ = false;
  Vtime_ = 0;
  VtnowStart_ = 0;
};

bool Calibrate(void)
{
  int throttle = 180;
  while ( throttle>0 )
  {
    myservo.write(throttle);
    throttle -= 10;
    delay(50);      
  }
  return(true);
}

// Discuss things with user
void talk(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,
    int *buttonState, ControlLaw *CLAW, const int potThrottle)
{
  // Serial event  (terminate Send String data with 0A using CoolTerm)
  double potThrottleX;
  if (stringComplete)
  {
    switch ( inputString.charAt(0) )
    {
      case ( 'p' ):
        plotting = !plotting;
        break;
      case ( 'S' ):
        switch ( inputString.charAt(1) )
        {
          case ( 'd' ):
            CLAW->Sd(inputString.substring(2).toFloat());
            break;
          case ( 'g' ):
            CLAW->Sg(inputString.substring(2).toFloat());
            break;
          case ( 't' ):
            CLAW->St(inputString.substring(2).toFloat());
            break;
        }
        break;
      case ( 'A' ):
        switch ( inputString.charAt(1) )
        {
          case ( 'd' ):
            CLAW->Ad(inputString.substring(2).toFloat());
            break;
          case ( 'g' ):
            CLAW->Ag(inputString.substring(2).toFloat());
            break;
          case ( 't' ):
            CLAW->At(inputString.substring(2).toFloat());
            break;
        }
        break;
      case ( 'f' ):
        if ( ttype==1 ) analyzer->complete(freqResp); // reset if doing freqResp
        freqResp = !freqResp;
        break;
      case ( 'V' ):
        if ( ttype==2 ) Vcomplete(*vectoring); // reset if doing vector
        *vectoring = !(*vectoring);
        break;
      case ( 'R' ):
        if ( ttype==3 ) Rcomplete(*vectoring); // reset if doing ramp
        *vectoring = !(*vectoring);
        break;
      case ( 'X' ):
        if ( ttype==4 ) RandComplete(*vectoring); // reset if doing vector
        *vectoring = !(*vectoring);
        break;
      case ( 'b' ):
        bare = !bare;
        break;
      case ( 'B' ):
        *buttonState = 1;
        break;
      case ( 'd' ):
        dry = !dry;
        break;
      case ( 'c' ):
        *closingLoop = !(*closingLoop);
        break;
      case ( 's' ):
        *stepping = !(*stepping);
        stepVal = -stepVal;
        break;
      case ( 'v' ):
        verbose = fmax(fmin(inputString.substring(1).toInt(), 10), 0);
        break;
      case ( 'P' ):
        potThrottleX = inputString.substring(1).toFloat();
        if (potThrottleX>=THTL_MIN && potThrottleX<=THTL_MAX)  // ignore otherwise
        {
          double vpotX = potThrottleX * POT_MAX / THTL_MAX;
          *potValue  = (vpotX*POT_SCL + POT_BIA)/POT_MAX*INSCALE;
        } 
        if (potThrottleX > -50) potOverride = true;
        else potOverride = false;
        break;
      case ( 'T' ):
        switch ( inputString.charAt(1) )
        {
          case ( 's' ):
            ttype = 0;  // Step
            break;
          case ( 'f' ):
            ttype = 1;  // Freq
            break;
          case ( 'v' ):
            ttype = 2;  // Vect
            break;
          case ( 'r' ):
            ttype = 3;  // Ramp
            break;
          case ( 'x' ):
            ttype = 4;  // Random
            break;
          case ( 'q' ):
            ttype = 5;  // Square at step amplitude
            break;
        }
        switch (ttype)
        {
          case (0):
            testOnButton = STEP;
            break;
          case (1):
            testOnButton = FREQ;
            break;
          case (2):
            testOnButton = VECT;
            break;
          case (3):
            testOnButton = RAMP;
            break;
          case (4):
            testOnButton = RAND;
            break;
          case (5):
            testOnButton = SQUARE;
            break;
//          otherwise:
  //          testOnButton = STEP;
      }
      break;
      case ('h'):
        Serial.print("p= "); Serial.print(plotting);    Serial.println("    : plotting out SerialUSB [on]");
        Serial.print("Sd= "); Serial.print(CLAW->Sd());  Serial.println("    : PID derivative scalar [1]");
        Serial.print("Ad= "); Serial.print(CLAW->Ad());  Serial.println("    : PID derivative adder [1]");
        Serial.print("          tld= "); Serial.print(CLAW->tldF()); Serial.println("    : PID fixed lead [0.015]");
        Serial.print("          tld= "); Serial.print(CLAW->tlgF()); Serial.println("    : PID fixed lag [0.015]");
        Serial.print("Sg= "); Serial.print(CLAW->Sg());  Serial.println("    : PID loopgain scalar [1]");
        Serial.print("Ag= "); Serial.print(CLAW->Ag());  Serial.println("    : PID loopgain adder [1]");
        Serial.print("St= "); Serial.print(CLAW->St());  Serial.println("    : PID lead (proportional) scalar [1]");
        Serial.print("At= "); Serial.print(CLAW->At());  Serial.println("    : PID lead (proportional) adder [1]");
        Serial.print("f=  "); Serial.print(freqResp);    Serial.println("    : frequency response test toggle [false]");
        Serial.print("V=  "); Serial.print(*vectoring);  Serial.println("    : vectoring toggle [false]");
        Serial.print("R=  "); Serial.print(*vectoring);  Serial.println("    : ramping toggle [false]");
        Serial.print("X=  "); Serial.print(*vectoring);  Serial.println("    : random toggle [false]");
        Serial.print("b=  "); Serial.print(bare);        Serial.println("    : bare hardware toggle [false]");
        Serial.print("B=  "); Serial.print(*buttonState);Serial.println("    : button state toggle [false]");
        Serial.print("d=  "); Serial.print(dry);         Serial.println("    : dry toggle no turbine [false]");
        Serial.print("c=  "); Serial.print(*closingLoop);Serial.println("    : closing loop toggle [false]");
        Serial.print("s=  "); Serial.print(*stepping);   Serial.println("    : stepping toggle [false]");
        Serial.print("v=  "); Serial.print(verbose);     Serial.println("    : verbosity, 0-10 [0]");
        Serial.print("P=  "); Serial.print(potThrottle); Serial.println("    : POT override, deg [when input P value>-50]");
        Serial.print("T=  "); 
        switch (ttype)
        {
          case (0):
            Serial.print("STEP");
            break;
          case (1):
            Serial.print("FREQ");
            break;
          case (2):
            Serial.print("VECT");
            break;
          case (3):
            Serial.print("RAMP");
            break;
          case (4):
            Serial.print("RAND");
            break;
          case (5):
            Serial.print("SQUARE");
            break;
      }
      Serial.println("    : transient performed with button push [STEP]");
    }
    inputString = "";
    stringComplete = false;
  }
}

