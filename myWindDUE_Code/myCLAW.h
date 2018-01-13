#ifndef _myCLAW_h
#define _myCLAW_h

#include "myTables.h"
#include "myFilters.h"
#include "myPID.h"


// Plant
static const double tau = 0.10;     // Input noise filter time constant, sec
static const double tldV = 0.0;     // Model F2V lead time constant, sec
static const double tauF2V = 0.1;   // Model F2V lag time constant, sec
static const double tldG = 0.0;     // Model Gas Generator lead time constant, sec
static const double tauG = 0.1;     // Model Gas Generator lag time constant, sec
static const double tldT = 0.0;     // Model Turbine lead time constant, sec
static const double NG_MIN = 0;     // Minimum trim, %Ng
static const double RATE_MAX = 240; // Maximum throttle change rate, deg/sec to avoid lockout
// See calibration<date>.xlsx for these hardware conversion derivations
static const double RPM_P = 461;                   // (rpm/%)
static const double THTL_MIN = 0;   // Minimum throttle, deg
static const double AREA_SI = 0.002121;            // Flow area, m^2
static const double NM_2_FTLBF = 0.738;            // ft-lbf/N-m conversion
static const double J = 3.743e-8;   // Turbine inertia, (rpm/s)/(ft-lbf)
static const double D_SI = 0.055;                  // Turbine dia, m
static const double FG_SI = 4.425;                 // Thrust at rated speed, N
static const double THTL_MAX = 180; // Maximum throttle, deg
static const double NG_MAX = 100;   // Maximum trim, %Ng

// Lookups where accuracy less of an issue
static const double P_LTALL_NG[2] = {-25454, 13062};  // Common coeff throttle(deg) to NG(rpm)
static const double P_NGALL_NT[2] = {-7208, 1.0000};  // Coeff NG(rpm) to NT(rpm)
static const double P_NTALL_NG[2] = {-7208, 1.0000};  // Coeff NG(rpm) to NT(rpm)
//
// Generic throttle-->Ng used in control laws.   These characteristics below (except xALL) are for test consistency.
// xALL nominally used to calibrate for a wildly different tauT on some kits.   Then problem can concentrate on gain variations.

// Coeff V4(v) to NT(rpm) normalized to 5 vdc
static const double P_V4_NT[8][3] = { {0, 13002, 298}, {0, 14111, 265}, {0, 14111, -114},
                                      {0, 12866, 327}, {0, 12773, 200}, {0, 13109, 399},
                                      {0, 13166, 308}, {0, 13848, -78}};

// F2Vs
// The actual 5 volt regulated value 
static const double V5vdc[6]      = { 4.94, 4.92, 4.93, 999, 4.89, 4.80 }; // 1/7/2018

// KITs
// Ard1_Turn0_ESC0_G0b_T0a 1/4/2017
// Ard1_Turn1x_ESC1_G1b_T1a 1/23/2017
// Ard2_Turn2_ESC2_G2b_T2a 1/24/2017
// Ard3_Turn3_ESC3_G3b_T3a 1/24/2017
// Ard4_Turn4_ESC4_G4b_T4a 1/24/2017
// Ard_Turn_ESC_Gb_Ta 1/24/2017
// Gain table lookups, Nt breakpoints
static const double xALL[6][6]  = { {0.,    0.01,  27.2,    47.4,   75.1,    80.},
                                    {0.,    21.7,  37.3,    50.5,   64.1,    80.},
                                    {0.,    22.0,  36.5,    49.1,   67.7,    80.},
                                    {0.,    21.6,  37.0,    51.0,   70.8,    80.},
                                    {0.,    21.6,  37.5,    51.3,   67.9,    80.},
                                    {0.,    14,   31.9,     48,     69.7,    80.}}; 
// Coeff NG(rpm) to NT(rpm)
static const double P_NG_NT[6][2]= { {-8053, 0.9475},  {-5671, 0.973}, {-5142, 0.919},
                                      {-5882, 0.950},   {-6435, 0.9771},{-8641, 0.9805}};
// TODO dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double DCPDL[6]     = { -0.418,   -0.89,  -1.1,   -0.83,  -1.3,   -1.4};
// TODO Turbine tip speed ratio to air velocity, dimensionless
static const double LAMBDA[6]    = { 2.54,     3.4,    3.6,    3.4,    4.0,    3.8};
// TODO Air velocity turbine first moves, m/s
static const double DELTAV[6]    = { 1,        7,      9,      8,      10,     11};
// POT
static const double POT_MIN[6]   = {0.01,     0.04,     0.01,   0.026,    0.01,     0.29};
static const double POT_MAX[6]   = {3.29,     3.29,     3.29,   3.29,     3.29,     3.29};

                                      

// Control Law Class
class ControlLaw
{
public:
  ControlLaw();
  ControlLaw(const double T, const double DENS_SI, const int myKit, const int myF2v);
  ~ControlLaw(){};
  // operators
  // functions
  double calculate(const int RESET, const double updateTime, const boolean closingLoop,
                   const boolean analyzing, const boolean freqResp, const boolean vectoring, 
                   const double exciter, const double freqRespScalar,
                   const double freqRespAdder, const double potThrottle, const double vf2v);
  double e(void) { return (e_); };
  double intState(void) { return (intState_); };
  double Ki(void) {return (Ki_); };
  double Kp(void) {return (Kp_); };
  double modelTS(void) { return (modelTS_); };
  double modelG(void) { return (modelG_); };
  double p(void) { return (p_); };
  double pcnt(void) { return (pcnt_); };
  double pcntRef(void) { return (pcntRef_); };
  int    throttle(void) { return (throttle_); };
  double Sd(void) { return(sd_);};  // Der Tld scalar
  double Sg(void) { return(sg_);};  // Gain scalar
  double Sl(void) { return(sl_);};  // Der Tlg scalar
  double St(void) { return(st_);};  // Prop Tld scalar
  double Ad(void) { return(ad_);}; 
  double Ag(void) { return(ag_);};
  double Al(void) { return(al_);}; 
  double At(void) { return(at_);};
  double tldF(void) { return(tldF_);};
  double tlgF(void) { return(tlgF_);};
  double LG(void) { return(Ki_);};
  int myKit(const int kit) { myKit_ = kit;};
  int myF2v(const int f2v) { myF2v_ = f2v;};
  int myKit(void) { return(myKit_);};
  int myF2v(void) { return(myF2v_);};
  double TLD(void) { return(Kp_/Ki_);};
  void Sd(const double S) { sd_ = S; };
  void Sg(const double S) { sg_ = S; };
  void Sl(const double S) { sl_ = S; };
  void St(const double S) { st_ = S; };
  void Ad(const double A) { ad_ = A; };
  void Ag(const double A) { ag_ = A; };
  void Al(const double A) { al_ = A; };
  void At(const double A) { at_ = A; };
private:
  double throttleLims(const int RESET, const double updateTime, const boolean closingLoop,
                  const boolean freqResp, const boolean vectoring, const double exciter, const double freqRespScalar,
                  const double freqRespAdder, const double potThrottle, const double ngmin);
  void model(const double throttle, const int RESET, const double updateTime);
  LeadLagExp *modelFilterG_; // Exponential lag model gas gen
  LeadLagExp *modelFilterT_; // Exponential lag model turbine
  LeadLagExp *modelFilterV_; // Exponential lag model F2V sensor
  TableInterp1Dclip *LG_T_;  // Gain schedule lead time constant, s
  TableInterp1Dclip *TLD_T_; // Gain schedule loop gain, r/s
  double DENS_SI_;           // Air density, kg/m^3
  double ad_;                // Derivative tlead adder
  double ag_;                // Integral lookup adder
  double al_;                // Derivative tlag adder
  double at_;                // Proportional lookup adder
  double dQ_;                // Precalculated coefficient, N-m/rpm/(m/s)
  double e_;                 // Closed loop error, %Nt
  double intState_;          // PI control integrate state, deg
  double Ki_;                // Integral gain, r/s
  double Kp_;                // Proportional gain, rad
  double modelG_;            // Model Gas Generator output, %Ng
  double modelT_;            // Model Turbine, %Nt
  double modelTS_;           // Model Turbine Sensed, %Nt
  double modPcng_;           // Modeled pcng ref after esc ttl delay, %Nt
  int    myF2v_;             // F2V serial number personality index
  int    myKit_;             // Kit serial number personality index
  double p_;                 // Prop path, %Nt
  double pcnt_;              // Turbine speed, %
  double pcntRef_;           // Turbine speed closed loop reference, %Nt
  double sd_;                // Derivative tlead lookup scalar
  double sg_;                // Integral lookup scalar
  double sl_;                // Derivative tlag lookup scalar
  double st_;                // Proportional lookup scalar
  double throttle_;          // Final value sent to hardware and model, deg
  double throttleL_;         // Limited servo value, 0-179 degrees
  double throttleCL_;        // Closed loop throttle output, deg
  double tldF_;              // Fixed lead, sec
  double tlgF_;              // Fixed lag, sec
  LeadLagExp *clawFixedL_;   // Exponential control fixed lead lag
};

#endif

