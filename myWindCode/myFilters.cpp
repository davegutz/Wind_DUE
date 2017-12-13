/***************************************************
  A simple dynamic filter library

  Class code for embedded application.

  07-Jan-2015   Dave Gutz   Created
  30-Sep-2016   Dave Gutz   LeadLagTustin
  23-Nov-2016   Dave Gutz   LeadLagExp
 ****************************************************/
#include "myFilters.h"
#include "math.h"

#ifndef ARDUINO
#include "application.h" // Should not be needed if file .ino or Arduino
#endif

extern int verbose;

// class DiscreteFilter
// constructors
DiscreteFilter::DiscreteFilter()
    : max_(1e32), min_(-1e32), rate_(0.0), T_(1.0), tau_(0.0) {}
DiscreteFilter::DiscreteFilter(const double T, const double tau, const double min, const double max)
    : max_(max), min_(min), rate_(0.0), T_(T), tau_(tau) {}
DiscreteFilter::~DiscreteFilter() {}
// operators
// functions
double DiscreteFilter::calculate(double input, int RESET)
{
  if (RESET > 0)
  {
    rate_ = 0.0;
  }
  return (rate_);
}
void DiscreteFilter::rateState(double in) {}
double DiscreteFilter::rateStateCalc(double in) { return (0); }
void DiscreteFilter::assignCoeff(double tau) {}
double DiscreteFilter::state(void) { return (0); }

// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
// constructors
RateLagTustin::RateLagTustin() : DiscreteFilter() {}
RateLagTustin::RateLagTustin(const double T, const double tau, const double min, const double max)
    : DiscreteFilter(T, tau, min, max)
{
  RateLagTustin::assignCoeff(tau);
}
//RateLagTustin::RateLagTustin(const RateLagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
RateLagTustin::~RateLagTustin() {}
// operators
// functions
double RateLagTustin::calculate(double in, int RESET)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  RateLagTustin::rateState(in);
  return (rate_);
}
void RateLagTustin::rateState(double in)
{
  rate_ = fmax(fmin(a_ * (in - state_), max_), min_);
  state_ = in * (1.0 - b_) + state_ * b_;
}
void RateLagTustin::assignCoeff(double tau)
{
  tau_ = tau;
  a_ = 2.0 / (2.0 * tau_ + T_);
  b_ = (2.0 * tau_ - T_) / (2.0 * tau_ + T_);
}
double RateLagTustin::state(void) { return (state_); };

// Tustin lead-lag alculator, non-pre-warped, no limits, fixed update rate
// constructors
LeadLagTustin::LeadLagTustin() : DiscreteFilter() {}
LeadLagTustin::LeadLagTustin(const double T, const double tld, const double tau, const double min, const double max)
    : DiscreteFilter(T, tau, min, max)
{
  LeadLagTustin::assignCoeff(tld, tau, T);
}
//LeadLagTustin::LeadLagTustin(const LeadLagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
LeadLagTustin::~LeadLagTustin() {}
// operators
// functions
double LeadLagTustin::calculate(double in, int RESET)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  double out = LeadLagTustin::rateStateCalc(in);
  return (out);
}
double LeadLagTustin::calculate(double in, int RESET, const double T, const double tau, const double tld)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  LeadLagTustin::assignCoeff(tld, tau, T);
  double out = LeadLagTustin::rateStateCalc(in, T);
  return (out);
}
double LeadLagTustin::calculate(double in, int RESET, const double T)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  double out = LeadLagTustin::rateStateCalc(in, T);
  return (out);
}
double LeadLagTustin::rateStateCalc(const double in)
{
  double out = rate_ + state_;
  rate_ = fmax(fmin(b_ * (in - state_), max_), min_);
  state_ = in * (1.0 - a_) + state_ * a_;
  return (out);
}
double LeadLagTustin::rateStateCalc(const double in, const double T)
{
  assignCoeff(tld_, tau_, T);
  double out = rateStateCalc(in);
  return (out);
}
void LeadLagTustin::assignCoeff(const double tld, const double tau, const double T)
{
  T_ = T;
  tld_ = tld;
  tau_ = tau;
  a_ = (2.0 * tau - T_) / (2.0 * tau_ + T_);
  b_ = (2.0 * tld_ + T_) / (2.0 * tau_ + T_);
}
double LeadLagTustin::state(void) { return (state_); };

// Exponential lead-lag calculator, non-pre-warped, no limits, fixed update rate
// http://www.mathpages.com/home/kmath198/2-2/2-2.htm
// constructors
LeadLagExp::LeadLagExp() : DiscreteFilter() {}
LeadLagExp::LeadLagExp(const double T, const double tld, const double tau, const double min, const double max)
    : DiscreteFilter(T, tau, min, max)
{
  LeadLagExp::assignCoeff(tld, tau, T);
}
//LeadLagExp::LeadLagExp(const LeadLagExp & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
LeadLagExp::~LeadLagExp() {}
// operators
// functions
double LeadLagExp::calculate(double in, int RESET)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  double out = LeadLagExp::rateStateCalc(in);
  return (out);
}
double LeadLagExp::calculate(double in, int RESET, const double T, const double tau, const double tld)
{
  if (RESET > 0)
  {
    instate_ = in;
    state_ = in;
  }
  LeadLagExp::assignCoeff(tld, tau, T);
  double out = LeadLagExp::rateStateCalc(in, T);
  return (out);
}
double LeadLagExp::calculate(double in, int RESET, const double T)
{
  if (RESET > 0)
  {
    instate_ = in;
    state_ = in;
  }
  double out = LeadLagExp::rateStateCalc(in, T);
  return (out);
}
double LeadLagExp::rateStateCalc(const double in)
{
  rate_ = fmax(fmin(b_ * (in - instate_), max_), min_);
  state_ += (a_ * (instate_ - state_) + rate_);
  instate_ = in;
  return (state_);
}
double LeadLagExp::rateStateCalc(const double in, const double T)
{
  assignCoeff(tld_, tau_, T);
  double out = rateStateCalc(in);
  return (out);
}
void LeadLagExp::assignCoeff(const double tld, const double tau, const double T)
{
  T_   = fmax(T, 1e-9);
  tld_ = fmax(tld, 0.0);
  tau_ = fmax(tau, 0.0);
  if (tau_ > 0.)  a_ = 1.0 - exp(-T_ / tau_);
  else            a_ = 1.0;
  b_ = 1.0 + a_ * (tld_ - tau_) / T_;
}
double LeadLagExp::state(void) { return (state_); };

// Exponential rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
// constructors
RateLagExp::RateLagExp() : DiscreteFilter() {}
RateLagExp::RateLagExp(const double T, const double tau, const double min, const double max)
    : DiscreteFilter(T, tau, min, max)
{
  RateLagExp::assignCoeff(tau);
}
//RateLagExp::RateLagExp(const RateLagExp & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
RateLagExp::~RateLagExp() {}
// operators
// functions
double RateLagExp::calculate(double in, int RESET)
{
  if (RESET > 0)
  {
    lstate_ = in;
    rstate_ = in;
  }
  RateLagExp::rateState(in);
  return (rate_);
}
double RateLagExp::calculate(double in, int RESET, const double T)
{
  if (RESET > 0)
  {
    lstate_ = in;
    rstate_ = in;
  }
  RateLagExp::rateState(in, T);
  return (rate_);
}
void RateLagExp::rateState(double in)
{
  rate_ = fmax(fmin(c_ * (a_ * rstate_ + b_ * in - lstate_), max_), min_);
  rstate_ = in;
  lstate_ += T_ * rate_;
}
void RateLagExp::rateState(double in, const double T)
{
  T_ = T;
  assignCoeff(tau_);
  rateState(in);
}
void RateLagExp::assignCoeff(double tau)
{
  double eTt = exp(-T_ / tau_);
  a_ = tau_ / T_ - eTt / (1 - eTt);
  b_ = 1.0 / (1 - eTt) - tau_ / T_;
  c_ = (1.0 - eTt) / T_;
}
double RateLagExp::state(void) { return (lstate_); };

// Tustin lag calculator, non-pre-warped, no limits, fixed update rate
// constructors
LagTustin::LagTustin() : DiscreteFilter() {}
LagTustin::LagTustin(const double T, const double tau, const double min, const double max)
    : DiscreteFilter(T, tau, min, max)
{
  LagTustin::assignCoeff(tau);
}
//LagTustin::LagTustin(const LagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
LagTustin::~LagTustin() {}
// operators
// functions
double LagTustin::calculate(double in, int RESET)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  LagTustin::calcState(in);
  return (state_);
}
double LagTustin::calculate(double in, int RESET, const double T)
{
  if (RESET > 0)
  {
    state_ = in;
  }
  LagTustin::calcState(in, T);
  return (state_);
}
void LagTustin::calcState(double in)
{
  rate_ = fmax(fmin(a_ * (in - state_), max_), min_);
  state_ = in * (1.0 - b_) + state_ * b_;
}
void LagTustin::calcState(double in, const double T)
{
  T_ = T;
  assignCoeff(tau_);
  calcState(in);
}
void LagTustin::assignCoeff(double tau)
{
  tau_ = tau;
  a_ = 2.0 / (2.0 * tau_ + T_);
  b_ = (2.0 * tau_ - T_) / (2.0 * tau_ + T_);
}
double LagTustin::state(void) { return (state_); };
