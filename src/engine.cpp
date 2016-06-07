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
int   delay = 10;

void GetCurrentSteer() {
    uint32_t line, column, index = 0;
    uint16_t TFC_LineScanImage_bin[CAMERA_BUFFER_SIZE];
    uint16_t leftAccumulate = 0, rightAccumulate = 0;
    float difference;

    TFC_SetBatteryLED_Level(4);

    for(line = 0; line < 8; ++line) { // print one line worth of data (128) from Camera 0
        for(column = 0; column < 16; ++column) {

            if (TFC_LineScanImage0[(line * 16) + column] > 0x250) {
                TFC_LineScanImage_bin[line * 16 + column] = 0x01;
            }
            else {
                TFC_LineScanImage_bin[line * 16 + column] = 0x00;
            }
            TERMINAL_PRINTF("%X", TFC_LineScanImage_bin[(i * 16) + j]);
        }
    }


    for(index = 0; index < CAMERA_BUFFER_MID; ++index) {
        leftAccumulate += TFC_LineScanImage_bin[index];
        //TERMINAL_PRINTF("%i", TFC_LineScanImage_bin[index]);
    }
    do {
        rightAccumulate += TFC_LineScanImage_bin[index];
        //TERMINAL_PRINTF("%i", TFC_LineScanImage_bin[index]);
        ++index;
    } while(index < CAMERA_BUFFER_SIZE);

    TERMINAL_PRINTF("Left: %i\r\n", leftAccumulate);
    TERMINAL_PRINTF("Right: %i\r\n", rightAccumulate);
    TERMINAL_PRINTF("Index: %i\r\n", index);

    difference = rightAccumulate - leftAccumulate;
    CURRENT_STEER = difference * 1.5 + DEFAULT_STEER; 
}

void SetMaxSpeed() {
    MAX_SPEED = TFC_ReadPot(0);
}

void SetSteer() {
    GetCurrentSteer();

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

        RUNNING |= 0x01;
        TFC_HBRIDGE_ENABLE;
        TFC_SetMotorPWM(MAX_SPEED, MAX_SPEED);
    }

    if(TFC_PUSH_BUTTON_1_PRESSED) {
        TFC_BAT_LED0_OFF;
        TFC_BAT_LED1_OFF;
        TFC_BAT_LED2_OFF;
        TFC_BAT_LED3_OFF;

        RUNNING &= ~0xFF;
        TFC_SetMotorPWM(0, 0);
        TFC_HBRIDGE_DISABLE;
    }

    if(TFC_LineScanImageReady && RUNNING) {
        // Disarm LineScanImageReady
        TFC_LineScanImageReady &= ~0xFF;
        SetSteer();
    }
}
