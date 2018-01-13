//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#include <Servo.h>
#include <Arduino.h>     // Used instead of Print.h - breaks Serial
#ifdef __AVR_ATmega328P__
  #define UNO
#endif
#ifdef __AVR_ATmega2560__
  #define MEGA
#endif
#ifdef __AVR_ATmega2561__
  #define MEGA
#endif
#ifdef __SAM3X8E__
  #define DUE
#endif

#include "myAnalyzer.h"
#include "myTables.h"
#include "math.h"

// Test features
int     verbose = 0;     // [0] Debug, as much as you can tolerate.   For talk() set using "v#"
bool    bare = false;    // [false] The microprocessor is completely disconnected.  Fake inputs and sensors for test purposes.  For talk() set using "b"
bool    potOverride = false; // [false] The pot is over-ridden by talk()
bool    closeOverride = false; // [false] the close loop switch is over-ridden by talk();
bool    dry = false;     // [false] The turbine and ESC are disconnected.  Fake inputs and sensors for test purposes.  For talk() set using "t"
double  stepVal = 6;     // [6] Step input, %nf.  Try to make same as freqRespAdder
bool    plotting = true; // [false] This is for Serial Plotter compatible output (menu - Tools - Serial Plotter)
int     myKit  = 2;      // [0] Kit serial number for personality match
int     myF2v  = 2;      // [0] F2v serial number for personality match
bool    calibrating = false; // [false] passing through raw voltages so POT_MAX and POT_MIN can be determined

/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz


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

//********constants for all*******************
#define PUBLISH_DELAY    15000UL        // Time between cloud updates (), micros
#define CONTROL_DELAY    15000UL        // Control law wait (), micros
#define CONTROL_08_DELAY  CONTROL_DELAY*8UL    // Control law wait (), micros
#define CONTROL_100_DELAY CONTROL_DELAY*100UL  // Control law wait (), micros
#define FR_DELAY 4000000UL    // Time to start FR, micros
const double F2V_MIN = 0.0;   // Minimum F2V value, vdc
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
DetectRise *ButtonRise;   // Pushbutton leading edge
TFDelay  *EnableDelayed;  // Power wait for Serial turn on
TFDelay  *PowerDelayed;   // ESC wait for boot
SRLatch  *ThrottleHold;   // Wait for cal complete and throttle low to release throttle to hardware
String inputString = "";        // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete

// Test vector setup (functions at bottom of this file)
bool VectComplete(void);
double VectCalculate(double);
void VectReset(bool);
const double Vtv_[] =  {0,  8,  16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104,  112,  120,  128,  136, 144, 152, 156}; // Time, s
const double Vvv_[] =  {10, 20, 25, 30, 42, 48, 56, 62, 77, 90, 96, 90, 77, 62,   56,   48,   42,   30,  25,  20,  10};  // Excitation
const unsigned int Vnv_ = sizeof(Vtv_)/sizeof(double);  // Length of vector
double Voutput_ = 0;        // Excitation value
double Vtime_ = 0;          // Time into vector, s
double VtnowStart_ = 0;     // now time of vector start reference, s
bool VectComplete_ = false;    // Status of vector, T=underway
unsigned int Viv_ = 0;      // Index of present time in vector


// Ramp vector setup (functions at bottom of this file)
bool RampComplete(void);
double RampCalculate(double);
void RampReset(bool);
const double Rtv_[] =  {0,  8,  68, 78, 138, 148}; // Time, s
const double Rvv_[] =  {10, 10, 96, 96, 10,  10};  // Excitation
const unsigned int Rnv_ = sizeof(Rtv_)/sizeof(double);  // Length of vector
double Routput_ = 0;        // Excitation value
double Rtime_ = 0;          // Time into vector, s
double RtnowStart_ = 0;     // now time of vector start reference, s
bool RampComplete_ = false;    // Status of vector, T=underway
unsigned int Riv_ = 0;      // Index of present time in vector

// Calibration
bool Calibrate(void);
void talk(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,
    bool *softButton, ControlLaw *CLAW, const int potThrottle, const bool calComplete, const bool analyzing,
    double *stepVal, unsigned long *squareDelay );

void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
#endif
  pinMode(POWER_IN_PIN, INPUT);
  pinMode(POWER_EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteResolution(12);   // 4096 scaling, 255 if comment this out
  Serial.begin(115200);
  #ifdef DUE
    SerialUSB.begin(115200);
  #endif
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
    if (calibrating)
      sprintf(buffer, "\ntime,mode,vpot,vf2v,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
    else
      sprintf(buffer, "\ntime,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
    Serial.print(buffer);
  }

  // Serial Event to allow switches to be passed in by user Serial transmit
  inputString.reserve(200); // Reserve 200 bytes for inputString Serial Event

  // Instatiate gain scheduling tables
  CLAW = new ControlLaw(T, DENS_SI, myKit, myF2v);
  ClPinDebounce = new Debounce((digitalRead(CL_PIN) == HIGH), 3);
  PowerDebounce = new Debounce(false, 1);
  ButtonDebounce = new Debounce(0, 3);
  ButtonRise = new DetectRise();
  PowerDelayed  = new TFDelay(false, 0.12, 0.0, T*8);
  EnableDelayed = new TFDelay(false, 2.0,  0.0, T*8);
  ThrottleHold  = new SRLatch(true);
  delay(100);
}

void loop()
{
  static bool vectoring = false;          // Perform vector test status
  static bool powered = false;            // Monitor ESC power
  static bool powerToCal = false;         // Wait for ESC bootup
  static bool calComplete = false;        // Hardware completed calibration
  int buttonState = 0;                    // Pushbutton
  static bool softButton = false;         // Soft pushbutton always able to occur
  static bool buttonRose = false;         // Leading edge of button
  static bool closingLoop = false;        // Persisted closing loop by pin cmd, T/F
  static bool stepping = false;           // Step by Photon send String
  bool control;                           // Control frame, T/F
  bool control8;                          // Control 8T frame, T/F
  bool control100 ;                       // Control 100T frame, T/F
  bool controlSquare;                     // Control square wave frame, T/F
  bool publish;                           // Publish, T/F
  static bool analyzing;                         // Begin analyzing, T/F
  unsigned long now = micros();           // Keep track of time
  static unsigned long squareDelay = 5000000UL;   // [5000000] usec square wave change time
  static unsigned long start = 0UL;       // Time to start looping, micros
  double elapsedTime;                     // elapsed time, micros
  static double updateTime = 0.0;         // Control law update time, sec
  static unsigned long lastControl = 0UL;    // Last control law time, micros
  static unsigned long lastPublish = 0UL;    // Last publish time, micros
  static unsigned long lastControl8  = 0UL;  // Last control 10T time, micros
  static unsigned long lastControl100  = 0UL; // Last control 100T time, micros
  static unsigned long lastControlSquare = 0UL;    // Last control square wave time, micros
  static unsigned long lastButton = 0UL;  // Last button push time, micros
  static unsigned long lastFR = 0UL;      // Last analyzing, micros
  static int mode = 0;                    // Mode of operation First digit: bare, Second digit:  closingLoop, Third digit: testOnButton, Fourth digit:  analyzing
  static int RESET = 1;                   // Dynamic reset
  const double RESEThold = 5;             // RESET hold, s
  static double exciter = 0;              // Frequency response excitation, fraction
                                          ////////////////////////////////////////////////////////////////////////////////////
  static double vf2v = 0;                 // Converted sensed back emf LM2907 circuit measure, volts
  static double vf2v_filt = 0;            // Filtered f2v value, volts
  static double vpot_filt = 0;            // Pot value, volts
  static double vpotDead = 0;             // Sliding deadband value, volts
  static double vpot = 0;                 // Pot value, volts
  static int f2vValue = INSCALE / 4;      // Dial raw value
  static int potValue = INSCALE / 3;      // Dial raw value
  static double potThrottle = 0;

  // Executive
  if (start == 0UL) start = now;
  elapsedTime = double(now - start) * 1e-6;
  if (calibrating)
    publish = ((now - lastPublish) >= PUBLISH_DELAY*10 - CLOCK_TCK / 2);
  else
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
  unsigned long deltaTick8 = now - lastControl8 ;
  control8  = (deltaTick8 >= CONTROL_08_DELAY - CLOCK_TCK / 2);
  if (control8 )
  {
    lastControl8  = now;
  }
  unsigned long deltaTick100 = now - lastControl100 ;
  control100  = (deltaTick100 >= CONTROL_100_DELAY - CLOCK_TCK / 2);
  if (control100 )
  {
    lastControl100  = now;
  }
  unsigned long deltaTickSquare = now - lastControlSquare;
  controlSquare = (deltaTickSquare >= squareDelay - CLOCK_TCK / 2);
  if (controlSquare)
  {
    lastControlSquare = now;
    if ( testOnButton==SQUARE ) softButton = true;
  }
  if (!bare)
  {
    if ( control )
    {
      if ( !closeOverride ) closingLoop = ClPinDebounce->calculate(digitalRead(CL_PIN) == HIGH);
      buttonState = ButtonDebounce->calculate(digitalRead(BUTTON_PIN));
      buttonRose = ButtonRise->calculate(buttonState);
    }
    if ( control8  )
    {
      powerToCal = PowerDelayed->calculate(powered);
      powerEnable = EnableDelayed->calculate(true);
    }
    if ( control100  )
    {
      powered = PowerDebounce->calculate(digitalRead(POWER_IN_PIN) == HIGH);
    }
  }
  if ( (buttonRose||softButton) && (now - lastButton > 200000UL))
  {
    lastButton = now;
    softButton = false;
    if ( bare ) buttonState = LOW;  // Reset if bare
    stepping = false;  // Stop running step if not stepping
    switch ( testOnButton )
    {
      case FREQ:
      {
        analyzer->complete(freqResp); // reset if already doing freqResp
        freqResp = !freqResp;
        break;
      }
      case STEP: case SQUARE:
      {
        stepping  = true;
        stepVal  = -stepVal;
        break;
      }
      case VECT:
      {
        vectoring = !vectoring;
        VectReset(vectoring); // reset if already doing vector
        break;
      }
      case RAMP:
      {
        vectoring = !vectoring;
        RampReset(vectoring); // reset if already doing vector
        break;
      }
    }
  }
  switch (testOnButton)
  {
  case (FREQ):
    analyzing = ( ((now - lastFR) >= FR_DELAY && !analyzer->complete()) );
    break;
  case (VECT):
    analyzing = !VectComplete();
    break;
  case (RAMP):
    analyzing = !RampComplete();
    break;
  }
  mode = bare*10000 + closingLoop*1000 + dry*100 + testOnButton*10 + analyzing;

  // Discuss things with the user
  // When open interactive serial monitor such as CoolTerm
  // then can enter commands by sending strings.   End the strings with a real carriage return
  // right in the "Send String" box then press "Send."
  // String definitions are below.
  talk(&vectoring, &closingLoop, &stepping, &potValue, &softButton, CLAW,
      potThrottle, calComplete, analyzing, &stepVal, &squareDelay );

  // Interrogate analog inputs
  if (control)
  {
    if (!bare)
    {
      f2vValue = analogRead(F2V_PIN);
    }
    if (!bare && !potOverride)
    {
      potValue = analogRead(POT_PIN);
    }
    vf2v = double(f2vValue) / INSCALE * 3.3;
    if (calibrating)
    {
      vpot = (double(potValue) / INSCALE * 3.3);
    }
    else
    {
      double POT_BIA = POT_MIN[myKit]; 
      double POT_SCL = (POT_MAX[myKit] - POT_BIA) / 3.3;
      vpot = fmin(fmax((double(potValue) / INSCALE * 3.3 - POT_BIA) / POT_SCL, 0), 3.3);
    }
  }

  // Control law
  if (control)
  {
    vpotDead = fmax(fmin(vpotDead, vpot + vpotHalfDB), vpot - vpotHalfDB);
    if (!freqResp)
      vpot_filt = throttleFilter->calculate(vpotDead, RESET); // Freeze pot for FR
    potThrottle = vpot_filt * THTL_MAX / 3.3;      // deg
    double dNdT = P_LTALL_NG[1] / fmax(potThrottle, 1) / RPM_P;  // Rate normalizer, %Ng/deg
    potThrottle += stepping * stepVal / dNdT;

    throttle = CLAW->calculate(RESET, updateTime, closingLoop, analyzing, freqResp, vectoring, exciter, freqRespScalar, freqRespAdder, potThrottle, vf2v);
    if (elapsedTime > RESEThold)
      RESET = 0;
  }

  // Commands to Hardware
  if ( control )
  {
    if ( !dry && !bare )
    {
      // Calibration always happens if power status responds properly (powered returns after power enable).
      // When calibration complete, though, potThrottle must be below 5 degrees to release user throttle to hardware
      // Until throttle is pulled back, end of calibration (0 degrees) is held.
      // As soon as bare is detected, terminate hardware outputs for safety reasons.
      bool calibrate = powerToCal && !calComplete;
      if ( calibrate ) calComplete = Calibrate();
      else if ( calComplete )
      {
        bool freeze = ThrottleHold->calculate(!calComplete, calComplete && potThrottle<2);
        if ( freeze )
        {
          myservo.write(0);
          digitalWrite(LED_BUILTIN, HIGH);
        }
        else
        {
          myservo.write(throttle);
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
      if ( powerEnable )  digitalWrite(POWER_EN_PIN, HIGH);
      else digitalWrite(POWER_EN_PIN, LOW);
    }
    else  // Terminate hardware outputs for safety
    {
      myservo.write(0);
      digitalWrite(POWER_EN_PIN, LOW);
      calComplete = false;
      ThrottleHold->calculate(!calComplete, false); 
    }
    if (closingLoop && powered)
    {
      analogWrite(DAC0, (CLAW->pcntRef()   +50)*4095/200);
    }
    else
    {
      analogWrite(DAC0, (CLAW->modelTS()   +50)*4095/200);
    }
    analogWrite(  DAC1, (throttle          +0 )*4095/200);    
    //if ( !calComplete ){ Serial.print(calComplete);Serial.print(',');Serial.print(throttle);Serial.print(',');Serial.print(powerEnable);Serial.print(',');Serial.print(powered);Serial.print(',');Serial.print(powerToCal);Serial.println(',');}
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
      switch ( testOnButton )
      {
        case ( FREQ ):
          if ( freqResp ) exciter = analyzer->calculate(fn, nsigFn); // use previous exciter for everything
          break;
        case ( VECT ):
          if ( vectoring ) exciter = VectCalculate(elapsedTime);
          break;
        case ( RAMP ):
          if ( vectoring ) exciter = RampCalculate(elapsedTime);
          break;
      }
    }
  }

  // Publish results to serial bus
  if (publish)
  {
    #ifdef DUE
    if ( plotting )
    {
      SerialUSB.print(CLAW->pcntRef());SerialUSB.print(",");
      SerialUSB.print(CLAW->pcnt());   SerialUSB.print(",");
      SerialUSB.print(CLAW->modelTS());SerialUSB.print(",");
      SerialUSB.print(CLAW->modelG()); SerialUSB.print(",");
      SerialUSB.print(CLAW->throttle()/1.8); SerialUSB.print(",");
      SerialUSB.print(throttle/1.8);   SerialUSB.print(",");
      SerialUSB.println("");
    }
    #endif
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
          if ( testOnButton==FREQ ) analyzer->publish();
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
        Serial.print(vpot, 3);Serial.print(",");
        if (calibrating)
        {
          Serial.print(vf2v, 3);Serial.print(",");
        }
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
  switch ( testOnButton )
  {
    case (FREQ):
      if (analyzer->complete()) freqResp = false;
      break;
    case (VECT):
      if (VectComplete()) vectoring = false;
      break;
    case (RAMP):
      if (RampComplete()) vectoring = false;
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
double VectCalculate(const double tnow)
{
  if ( VtnowStart_ == 0 )
  {
    VtnowStart_ = tnow;  // First call sets time
    VectComplete_ = false;
    Viv_ = 0;
  }
  // Find location in vector
  Vtime_ = tnow-VtnowStart_;
  while ( Vtv_[Viv_]<Vtime_ && Viv_<Vnv_ ) Viv_++;
  // Output
  if ( Viv_ == Vnv_ ) VectComplete_ = true;
  unsigned int iv = Viv_;
  if ( iv==0 ) iv = 1;
  Voutput_ = Vvv_[iv-1];
  return ( Voutput_ );
};

bool VectComplete(void) { return (VectComplete_); };

// Restart vector
void VectReset(const bool set)
{
  Viv_ = 0;
  VectComplete_ = !set;
  Vtime_ = 0;
  VtnowStart_ = 0;
};



// Ramp calculator
double RampCalculate(const double tnow)
{
  if ( RtnowStart_ == 0 )
  {
    RtnowStart_ = tnow;  // First call sets time
    RampComplete_ = false;
    Riv_ = 0;
  }
  // Find location in vector
  Rtime_ = tnow-RtnowStart_;
  while ( Rtv_[Riv_]<Rtime_ && Riv_<Rnv_ ) Riv_++;   // iv is location past now
  // Output
  if ( Riv_ == Rnv_ ) RampComplete_ = true;
  unsigned int ir = Riv_;
  if ( ir==0 ) ir = 1;
  Routput_ = (Rtime_-Rtv_[ir-1]) / (Rtv_[ir]-Rtv_[ir-1]) * (Rvv_[ir]-Rvv_[ir-1])  +  Rvv_[ir-1];
  return ( Routput_ );
};

bool RampComplete(void) { return (RampComplete_); };

// Restart ramp
void RampReset(const bool set)
{
  RampComplete_ = !set;
  Riv_ = 0;
  Rtime_ = 0;
  RtnowStart_ = 0;
};


bool Calibrate(void)
{
  int throttle = 180;
  while ( throttle>=0 )
  {
    myservo.write(throttle);
    throttle -= 10;
    delay(50);      
  }
  return(true);}

// Discuss things with user

// Talk Declarations
void talkT(bool *stepping, double *stepVal, unsigned long *squareDelay );  // Transient Inputs
void talkh(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,  // Help
    bool *softButton, ControlLaw *CLAW, const int potThrottle, const bool calComplete, const bool analyzing,
    double *stepVal, unsigned long *squareDelay );

// Talk Executive
void talk(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,
    bool *softButton, ControlLaw *CLAW, const int potThrottle, const bool calComplete, const bool analyzing,
    double *stepVal, unsigned long *squareDelay )
{
  // Serial event  (terminate Send String data with 0A using CoolTerm)
  double potThrottleX;
  if (stringComplete)
  {
    switch ( inputString.charAt(0) )
    {
      case ( 'P' ):
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
      case ( 'B' ):
        bare = !bare;
        if (!bare) potOverride = false;
        break;
      case ( 'b' ):
        *softButton = true;
        break;
      case ( 'D' ):
        dry = !dry;
        break;
      case ( 'C' ):
        *closingLoop = !(*closingLoop);
        if ( *closingLoop ) closeOverride = true;
        else closeOverride = false;
        break;
      case ( 'X' ):
        calibrating = !calibrating;
        break;
      case ( 'v' ):
        verbose = fmax(fmin(inputString.substring(1).toInt(), 10), 0);
        break;
      case ( 'K' ):
        myKit = fmax(fmin(inputString.substring(1).toInt(), 10), 0);
        CLAW->myKit(myKit);
        break;
      case ( 'F' ):
        myF2v = fmax(fmin(inputString.substring(1).toInt(), 10), 0);
        CLAW->myF2v(myF2v);
        break;
      case ( 'p' ):
        potThrottleX = inputString.substring(1).toFloat();
        if (potThrottleX>=THTL_MIN && potThrottleX<=THTL_MAX)  // ignore otherwise
        {
          double vpotX = potThrottleX * POT_MAX[myKit] / THTL_MAX;
          double POT_BIA = POT_MIN[myKit]; 
          double POT_SCL = (POT_MAX[myKit] - POT_BIA) / 3.3;
          *potValue  = (vpotX*POT_SCL + POT_BIA)/3.3*INSCALE;
        } 
        if (potThrottleX > -50) potOverride = true;
        else potOverride = false;
        break;
      case ( 'T' ):
        talkT(stepping, stepVal, squareDelay);
        break;
      case ('H'):  // Headers
        if (calibrating)
          sprintf(buffer, "\nMONITOR:  time,mode,vpot,vf2v,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
        else
          sprintf(buffer, "\nMONITOR:  time,mode,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
        Serial.print(buffer);
        sprintf(buffer, "PLOTTING:  NtRef(blue), Nts(red), NtsM(green), NgM(orange), throttleU/1.8(magenta), throttle/1.8(olive)\n");
        Serial.print(buffer);
        break;
      case ('h'): 
        talkh(vectoring, closingLoop, stepping, potValue, softButton, CLAW, potThrottle, calComplete, analyzing, stepVal, squareDelay);
        break;
      default:
        Serial.print(inputString.charAt(0)); Serial.println(" unknown");
        break;
    }
    inputString = "";
    stringComplete = false;
  }
}

// Talk Tranient Input Settings
void talkT(bool *stepping, double *stepVal, unsigned long *squareDelay )
{
  *stepping = false;
  switch ( inputString.charAt(1) )
  {
    case ( 's' ): 
      *stepping = true;
      testOnButton = STEP;
      if ( inputString.substring(2).length() )
        *stepVal = fabs(inputString.substring(2).toFloat());
      break;
    case ( 'f' ):
      testOnButton = FREQ;
      break;
    case ( 'v' ):
      testOnButton = VECT;
      VectReset(false);
      break;
    case ( 'r' ):
      testOnButton = RAMP;
      RampReset(false);
      break;
    case ( 'q' ):
      testOnButton = SQUARE;
      *stepping = true;
      if ( inputString.substring(2).length() )
        *squareDelay = fabs(inputString.substring(2).toInt());
      break;
    default:
      Serial.print(inputString); Serial.println(" unknown.  Try typing 'h'");
  }
}

// Talk Help
void talkh(bool *vectoring, bool *closingLoop, bool *stepping, int *potValue,
    bool *softButton, ControlLaw *CLAW, const int potThrottle, const bool calComplete, const bool analyzing,
    double *stepVal, unsigned long *squareDelay )
{
  Serial.print("K= ");  Serial.print(myKit);       Serial.println("    : KIT number[0]");
  Serial.print("F= ");  Serial.print(myF2v);       Serial.println("    : F2V number[0]");
  Serial.print("P= ");  Serial.print(plotting);    Serial.println("    : plotting out SerialUSB [1]");
  Serial.print("Sd= "); Serial.print(CLAW->Sd());  Serial.println("    : PID derivative tlead scalar [1]");
  Serial.print("Ad= "); Serial.print(CLAW->Ad());  Serial.println("    : PID derivative tlead adder [0]");
  Serial.print("Sl= "); Serial.print(CLAW->Sl());  Serial.println("    : PID derivative tlag scalar [1]");
  Serial.print("Al= "); Serial.print(CLAW->Al());  Serial.println("    : PID derivative tlag adder [0]");
  Serial.print("   ref:   tld= "); Serial.print(CLAW->tldF(),3); Serial.println("    : present PID fixed lead");
  Serial.print("   ref:   tlg= "); Serial.print(CLAW->tlgF(),3); Serial.println("    : present PID fixed lag");
  Serial.print("Sg= "); Serial.print(CLAW->Sg());  Serial.println("    : PID loopgain (integral) scalar [1]");
  Serial.print("Ag= "); Serial.print(CLAW->Ag());  Serial.println("    : PID loopgain (integral) adder [0]");
  Serial.print("          LG= "); Serial.print(CLAW->LG()); Serial.println("    : PID loop gain, r/s [various]");
  Serial.print("St= "); Serial.print(CLAW->St());  Serial.println("    : PID lead (proportional) tlead scalar [1]");
  Serial.print("At="); Serial.print(CLAW->At());  Serial.println("    : PID lead (proportional) tlead adder [0]");
  Serial.print("          TLD= "); Serial.print(CLAW->TLD()); Serial.println("    : PID TLD, sec [various]");
  Serial.print("B=  "); Serial.print(bare);        Serial.println("    : bare hardware toggle [0]");
  Serial.print("b=  "); Serial.print(*softButton);Serial.println("    : button state toggle [0]");
  Serial.print("D=  "); Serial.print(dry);         Serial.println("    : dry toggle no turbine [0]");
  Serial.print("C=  "); Serial.print(*closingLoop);Serial.println("    : closing loop toggle [0].   Will over-ride a failed open switch.");
  Serial.print("X=  "); Serial.print(calibrating);Serial.println("    : calibrating toggle [false].");
  Serial.print("v=  "); Serial.print(verbose);     Serial.println("    : verbosity, 0-10. 2 for save csv [0]");
  Serial.print("P=  "); Serial.print(plotting);    Serial.println("    : streaming to U.   Toggle to pause plots [1]");
  Serial.print("p=  "); Serial.print(potThrottle); Serial.println("    : POT override, deg [when input P value>-50]");
  Serial.print("T<?>=  "); 
  switch (testOnButton)
  {
    case (STEP):
      Serial.print("s");
      break;
    case (FREQ):
      Serial.print("f");
      break;
    case (VECT):
      Serial.print("v");
      break;
    case (RAMP):
      Serial.print("r");
      break;
    case (SQUARE):
      Serial.print("q");
      break;
  }
  Serial.println("    : transient performed with button push (?= <s>STEP, <f>FREQ, <v>VECT, <r>RAMP, <q>SQUARE) [s]");
  Serial.print("          :   stepVal="); Serial.println(*stepVal);
  Serial.print("          :   squareDelay="); Serial.println(*squareDelay);
  Serial.print("Using KIT# ");  Serial.println(CLAW->myKit());
  Serial.print("Using F2V# ");  Serial.println(CLAW->myF2v());
  Serial.print("Calibration complete status="); Serial.print(calComplete);
  Serial.print(", vectoring="); Serial.print(*vectoring);
  Serial.print(", stepping=");  Serial.print(*stepping);
  Serial.print(", analyzing="); Serial.println(analyzing);
  Serial.print("MODE[bare|closingLoop|dry|testOnButton|analyzing]="); Serial.println(bare*10000+(*closingLoop)*1000+dry*100+testOnButton*10+analyzing);
}
    
