#include "mbed.h"

#include "common.h"

Serial PC(USBTX,USBRX); 

Ticker TFC_TickerObj;

volatile uint32_t TFC_Ticker[NUM_TFC_TICKERS];