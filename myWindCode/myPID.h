#ifndef _myPID_h
#define _myPID_h

// MatlabTuning20170213, PID with FIXED LEAD
static const double fixedLead = 0.15;                                           // Lead, s
static const double fixedLag  = 0.03;                                           // Lag, s
static const double yLG[6]  = {3.6,   3.6,    3.82,   4.15,   5.01,   5.01};    // Loop gain, r/s
static const double yTLD[6] = {0.429, 0.429,  0.242,  0.138,  0.093,  0.093}; // Lead, s

#endif
