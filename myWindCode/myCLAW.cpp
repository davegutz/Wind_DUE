// Standard
#include <Arduino.h> //needed for Serial.println
#include "myCLAW.h"
#include "math.h"

/* TODO
  1.  
  2.

*/

// Global variables
extern int verbose;
extern bool bare; // Fake inputs and sensors for test purposes
extern bool dry; // Fake inputs and sensors for test purposes
extern char buffer[256];

//Class ControlLaw
ControlLaw::ControlLaw()
    : DENS_SI_(0), ad_(0), ag_(0), at_(0), dQ_(0), intState_(0),
      modelG_(0), modelT_(0), modelTS_(0), modPcng_(0),
      pcnt_(0), pcntRef_(0), sd_(1), sg_(1), st_(1), 
      throttle_(0), throttleL_(0)
{
  LG_T_ = new TableInterp1Dclip(sizeof(xALL) / sizeof(double), xALL, yLG);
  TLD_T_ = new TableInterp1Dclip(sizeof(xALL) / sizeof(double), xALL, yTLD);
  tldF_ = fixedLead;
  tlgF_ = fixedLag;
  clawFixedL_ = new LeadLagExp(0, fixedLead, fixedLag, -1e6, 1e6);
  modelFilterG_ = new LeadLagExp(0, tldG, tauG, -1e6, 1e6);
  modelFilterT_ = new LeadLagExp(0, tldT, 1.00, -1e6, 1e6);
  modelFilterV_ = new LeadLagExp(0, tldV, tauF2V, -1e6, 1e6);
}
ControlLaw::ControlLaw(const double T, const double DENS_SI)
    : DENS_SI_(DENS_SI), ad_(0), ag_(0), at_(0), intState_(0),
      modelG_(0), modelT_(0), modelTS_(0), modPcng_(0),
      pcnt_(0), pcntRef_(0), sd_(1), sg_(1), st_(1), 
      throttle_(0), throttleL_(0)
{
  tldF_ = fixedLead;
  tlgF_ = fixedLag;
  clawFixedL_ = new LeadLagExp(T, tldF_, tlgF_, -1e6, 1e6);
  dQ_ = DCPDL * DENS_SI_ * D_SI * D_SI * AREA_SI * 3.1415926 / 240 / LAMBDA * NM_2_FTLBF;
  LG_T_ = new TableInterp1Dclip(sizeof(xALL) / sizeof(double), xALL, yLG);
  TLD_T_ = new TableInterp1Dclip(sizeof(xALL) / sizeof(double), xALL, yTLD);
  modelFilterG_ = new LeadLagExp(T, tldG, tauG, -1e6, 1e6);
  modelFilterT_ = new LeadLagExp(T, tldT, 1.00, -1e6, 1e6);
  modelFilterV_ = new LeadLagExp(T, tldV, tauF2V, -1e6, 1e6);
}

// Control Law
double ControlLaw::calculate(const int RESET, const double updateTime, const boolean closingLoop,
                             const boolean analyzing, const boolean freqResp, const boolean vectoring, 
                             const double exciter, const double freqRespScalar,
                             const double freqRespAdder, const double potThrottle, const double vf2v)
{
  static const double ngmin = NG_MIN-exp(-P_LTALL_NG[0]/P_LTALL_NG[1]);
  // Inputs
  if (bare || dry)
    pcnt_ = modelTS_;
  else
    pcnt_ = fmin(fmax(P_V4_NT[0] + vf2v * (P_V4_NT[1] + vf2v * P_V4_NT[2]) / RPM_P, 0.0), 100);
  // Add 1.15 in following line to drive to max
  double throttleRPM = fmin(fmax(1.15*(P_LTALL_NG[0] + P_LTALL_NG[1] * log(fmax(potThrottle, 1))), ngmin), RPM_P*100); // Ng demand at throttle, rpm
  pcntRef_ = (P_NG_NT[0] + P_NG_NT[1] * throttleRPM) / RPM_P;
  if (closingLoop && (freqResp || vectoring))
  {
    if ( freqResp )
    {
      pcntRef_ *= (1 + exciter / freqRespScalar);
      pcntRef_ += exciter * freqRespAdder;
    }
    else if ( vectoring )
    {
      pcntRef_ = exciter;
    }
  }

  // Initialization
  if (RESET)
  {
    intState_ = fmax((P_LTALL_NG[0] + 1.15*P_LTALL_NG[1] * log(potThrottle)) / RPM_P, ngmin);
    pcnt_ = pcntRef_;
    modelTS_ = pcntRef_;
  }

  // Control Law
  e_ = pcntRef_ - pcnt_;
  double ec = e_;
  Ki_ = LG_T_->interp(pcnt_)*sg_+ag_;                         // r/s
  Kp_ = (TLD_T_->interp(pcnt_)*st_+at_) * Ki_;                // %Ng/%Nf=1
  double dNdT = P_LTALL_NG[1] / fmax(potThrottle, 1) / RPM_P; // Rate normalizer, %Ng/deg
  double riMax = RATE_MAX * dNdT;

// PID
  ec = clawFixedL_->calculate(e_, RESET, updateTime, tlgF_, tldF_*sd_+ad_ );
  p_ = fmax(fmin(Kp_ * ec, NG_MAX), -NG_MAX);
  intState_ = fmax(fmin(intState_ + updateTime * fmax(fmin(Ki_ * ec, 0.5 * riMax), -0.5 * riMax), NG_MAX), -NG_MAX);
  double pcngCL = fmax(fmin(intState_ + p_, NG_MAX), ngmin);
  throttleCL_ = exp((pcngCL * RPM_P - P_LTALL_NG[0]) / P_LTALL_NG[1]);

  // Limits
  throttle_ = throttleLims(RESET, updateTime, closingLoop, freqResp, vectoring, exciter, freqRespScalar, freqRespAdder, potThrottle, ngmin);

  // Model
  model(throttle_, RESET, updateTime);

  return (throttle_);
}

// Rate limits
double ControlLaw::throttleLims(const int RESET, const double updateTime, const boolean closingLoop,
                                const boolean freqResp, const boolean vectoring, const double exciter, const double freqRespScalar, const double freqRespAdder,
                                const double potThrottle, const double ngmin)
{
  double throttle;
  double throttleU, stepChange;

  // Raw throttle calculation
  if (closingLoop) throttleU = throttleCL_;
  else
  {
     if ( freqResp) throttleU = potThrottle * (1 + exciter / freqRespScalar) + exciter * freqRespAdder; // deg throttle
     else if ( vectoring )
     {
        double throttleRPM = fmax(fmin((exciter*RPM_P - P_NG_NT[0])/P_NG_NT[1], NG_MAX*RPM_P), ngmin*RPM_P);
        throttleU = exp((throttleRPM - P_LTALL_NG[0]) / P_LTALL_NG[1]);
     }
     else throttleU = potThrottle;
  }
  throttleU = fmax(fmin(throttleU, THTL_MAX), THTL_MIN);

  // Apply rate limits as needed
  if (RESET) stepChange = 1 * updateTime;    // Maximum allowable step change reset, deg/update
  else stepChange   = RATE_MAX * updateTime; // Maximum allowable step change, deg/update
  throttleL_ = fmax(fmin(throttleU, throttleL_ + stepChange), throttleL_ - stepChange);

  // Final selection
  if (freqResp || !closingLoop) throttle = throttleU;
  else   throttle = throttleL_;

  return (throttle);
}

// Embedded model
void ControlLaw::model(const double throttle, const int RESET, const double updateTime)
{
  // Model
  double dQt_dNt; // Load line for inertia calculation, ft-lbf/rpm
  double tauT;    // Model Fan lag time constant, sec
  if (RESET)
    modPcng_ = fmin(fmax((P_NTALL_NG[0] + pcntRef_ * RPM_P * P_NTALL_NG[1]) / RPM_P, 0), 100);
  else
    modPcng_ = fmin(fmax(1.00*(P_LTALL_NG[0] + P_LTALL_NG[1] * log(double(int(throttle)))) / RPM_P, 0.0), 100);
  double Fg_SI = FG_SI * (modelG_/100)*(modelG_/100);     // N
  double Vw_SI = sqrt(Fg_SI / AREA_SI / DENS_SI_);        // m/s
  dQt_dNt = fmin(dQ_ * (Vw_SI - DELTAV), -1e-16);
  tauT = fmin(fmax(-J / dQt_dNt, 0.02), 0.5);
  modelG_ = modelFilterG_->calculate(modPcng_, RESET);
  modelT_ = modelFilterT_->calculate((P_NGALL_NT[0] + modelG_ * RPM_P * P_NGALL_NT[1]) / RPM_P, RESET, updateTime, tauT, tldT);
  modelTS_ = modelFilterV_->calculate(modelT_, RESET);
}

