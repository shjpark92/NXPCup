#include "mbed.h"

#ifndef _COMMON_H
#define _COMMON_H

// define commonly used global variables and constants here
extern Serial PC;

#define TERMINAL_PRINTF     PC.printf 

 //This ticker code is used to maintain compability with the Codewarrior version of the sample.   This code uses an MBED Ticker for background timing.
#define NUM_TFC_TICKERS 4

extern Ticker TFC_TickerObj;
extern volatile uint32_t TFC_Ticker[NUM_TFC_TICKERS];
#endif