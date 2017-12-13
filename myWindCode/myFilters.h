/***************************************************
  A simple dynamic filter library

  Class code for embedded application.

  07-Jan-2015   Dave Gutz   Created
  30-Sep-2016   Dave Gutz   LeadLagTustin
  23-Nov-2016   Dave Gutz   LeadLagExp
 ****************************************************/

#ifndef _myFilters_H
#define _myFilters_H

class DiscreteFilter
{
public:
  DiscreteFilter();
  DiscreteFilter(const double T, const double tau, const double min, const double max);
  virtual ~DiscreteFilter();
  // operators
  // functions
  virtual double calculate(double in, int RESET);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual double rateStateCalc(double in);
  virtual double state(void);

protected:
  double max_;
  double min_;
  double rate_;
  double T_;
  double tau_;
};

// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
class LeadLagTustin : public DiscreteFilter
{
public:
  LeadLagTustin();
  LeadLagTustin(const double T, const double tld, const double tau, const double min, const double max);
  //  LeadLagTustin(const LeadLagTustin & RLT);
  ~LeadLagTustin();
  //operators
  //functions
  virtual double calculate(const double in, const int RESET);
  virtual double calculate(const double in, const int RESET, const double T);
  virtual double calculate(double in, int RESET, const double T, const double tau, const double tld);
  virtual void assignCoeff(const double tld, const double tau, const double T);
  virtual double rateStateCalc(const double in);
  virtual double rateStateCalc(const double in, const double T);
  virtual double state(void);

protected:
  double a_;
  double b_;
  double state_;
  double tld_;
};

// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
class LeadLagExp : public DiscreteFilter
{
public:
  LeadLagExp();
  LeadLagExp(const double T, const double tld, const double tau, const double min, const double max);
  //  LeadLagExp(const LeadLagExp & RLT);
  ~LeadLagExp();
  //operators
  //functions
  virtual double calculate(const double in, const int RESET);
  virtual double calculate(const double in, const int RESET, const double T);
  virtual double calculate(double in, int RESET, const double T, const double tau, const double tld);
  virtual void assignCoeff(const double tld, const double tau, const double T);
  virtual double rateStateCalc(const double in);
  virtual double rateStateCalc(const double in, const double T);
  virtual double state(void);

protected:
  double a_;
  double b_;
  double state_;
  double instate_;
  double tld_;
};

// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
class RateLagTustin : public DiscreteFilter
{
public:
  RateLagTustin();
  RateLagTustin(const double T, const double tau, const double min, const double max);
  //  RateLagTustin(const RateLagTustin & RLT);
  ~RateLagTustin();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual double state(void);

protected:
  double a_;
  double b_;
  double state_;
};

// Exponential rate-lag rate calculator
class RateLagExp : public DiscreteFilter
{
public:
  RateLagExp();
  RateLagExp(const double T, const double tau, const double min, const double max);
  //RateLagExp(const RateLagExp & RLT);
  ~RateLagExp();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual double calculate(double in, int RESET, const double T);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual void rateState(double in, const double T);
  virtual double state(void);
  double a() { return (a_); };
  double b() { return (b_); };
  double c() { return (c_); };
  double lstate() { return (lstate_); };
  double rstate() { return (rstate_); };
protected:
  double a_;
  double b_;
  double c_;
  double lstate_; // lag state
  double rstate_; // rate state
};

// Tustin lag calculator
class LagTustin : public DiscreteFilter
{
public:
  LagTustin();
  LagTustin(const double T, const double tau, const double min, const double max);
  //  LagTustin(const LagTustin & RLT);
  ~LagTustin();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual double calculate(double in, int RESET, const double T);
  virtual void assignCoeff(double tau);
  virtual void calcState(double in);
  virtual void calcState(double in, const double T);
  virtual double state(void);
  double a() { return (a_); };
  double b() { return (b_); };
  double rate() { return (rate_); };
protected:
  double a_;
  double b_;
  double rate_;
  double state_;
};

#endif
