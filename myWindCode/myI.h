#ifndef _myI_h
#define _myI_h

// Simplified control Ref Step objective 11/19/2016
static const double tldF    = 0.015;                                           // Lead, s
static const double tlgF    = 0.015;                                           // Lag, s
static const double yLG[6]  = {2.4,   2.4,    2.7,    3.2,    3.75,   4.4};   // Loop gain, r/s
static const double yTLD[6] = {0.0,   0.0,    0.0,    0.0,    0.0,    0.0};   // Lead, s

#endif
