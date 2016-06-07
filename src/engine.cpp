#include "mbed.h"
#include "engine.h"
#include "common.h"
#include "TFC.h"

// Calibrated Values
#define MAX_STEER_LEFT    -0.63
#define MAX_STEER_RIGHT    0.22
#define MAX_STEER_RANGE    0.85
#define DEFAULT_STEER     -0.21
#define DT                 0.02
#define CAMERA_BUFFER_SIZE 128
#define CAMERA_BUFFER_MID  64

// TODO: Get rid of float??
float CURRENT_STEER = -0.21; // (MAX_STEER_RIGHT + MAX_STEER_LEFT) / 2
float MAX_SPEED     =  0.00;
int   RUNNING = 0;
int delay = 10;

void GetCurrentSteer() {
    uint32_t i, j;
    short TFC_LineScanImage_bin[CAMERA_BUFFER_SIZE];
    short leftAccumulate, rightAccumulate;
    float difference;

    if(TFC_Ticker[0] > 1000 && TFC_LineScanImageReady > 0) { // every 2s ...
        TFC_Ticker[0] = 0;
        TFC_LineScanImageReady = 0; // must reset to 0 after detecting non-zero
        TFC_SetBatteryLED_Level(4);

        for(i = 0; i < 8; i++) { // print one line worth of data (128) from Camera 0
            for(j = 0; j < 16; j++) {
                if (TFC_LineScanImage0[(i * 16) + j] > 0x110) {
                    TFC_LineScanImage_bin[i * 16 + j] = 1;
                }
                else {
                    TFC_LineScanImage_bin[i * 16 + j] = 0;
                }
                /*if((i == 7) && (j == 15)) {  // when last data reached put in line return
                //   TERMINAL_PRINTF("\r\n",TFC_LineScanImage0[(i * 16) + j]);
                }
                else {
                //   TERMINAL_PRINTF(", ",TFC_LineScanImage0[(i * 16) + j]);
                }*/
            }
        }
    }

    for(int i = 0; i < CAMERA_BUFFER_MID; ++i) {
        leftAccumulate += TFC_LineScanImage_bin[i];
    }
    do {
      rightAccumulate += TFC_LineScanImage_bin[i];
      ++i;
    } while(i < CAMERA_BUFFER_SIZE);

    difference = rightAccumulate - leftAccumulate;
    CURRENT_STEER = difference * 1.5 + DEFAULT_STEER; 
}

void SetMaxSpeed() {
    MAX_SPEED = TFC_ReadPot(0);
}

void SetSteer() {
    if(delay == 0) {
        GetCurrentSteer();
        delay = 10;
    }
    else {
        delay--;
    }

    // Set New Steer Value
    if (CURRENT_STEER > MAX_STEER_RIGHT) { 
        CURRENT_STEER = MAX_STEER_RIGHT;
    } 
    else if (CURRENT_STEER < MAX_STEER_LEFT) {
        CURRENT_STEER = MAX_STEER_LEFT;
    }
    else {
        CURRENT_STEER = DEFAULT_STEER;
    }

    TFC_BAT_LED0_OFF;
    TFC_SetServo(0, CURRENT_STEER);
}

void ExecuteEngine() {
    if(TFC_PUSH_BUTTON_0_PRESSED) {
        TFC_BAT_LED0_ON;
        TFC_BAT_LED1_ON;
        TFC_BAT_LED2_ON;
        TFC_BAT_LED3_ON;
        SetMaxSpeed();
        SetSteer();
        RUNNING = 1;
        TFC_HBRIDGE_ENABLE;
        TFC_SetMotorPWM(MAX_SPEED, MAX_SPEED);
    }

    if(TFC_PUSH_BUTTON_1_PRESSED) {
        TFC_BAT_LED0_OFF;
        TFC_BAT_LED1_OFF;
        TFC_BAT_LED2_OFF;
        TFC_BAT_LED3_OFF;

        TFC_SetMotorPWM(0, 0);
        TFC_HBRIDGE_DISABLE;
    }
    if (RUNNING) {
        SetSteer();
        TFC_BAT_LED0_ON;
    }

}
