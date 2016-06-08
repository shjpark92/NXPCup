#include "mbed.h"
#include "TFC.h"
#include "common.h"
#include "Spices.h"

// 5 MODES OR EXERCISES THAT WILL ALIGN WITH DIP SWITCH SETTINGS IN BINARY FASHION
//  e.g. Mode 1 = x001 <-- mode switch value representation
//                4321 <-- switch number
//
//  meaning: switch 1 is on, switch 2 is off, switch 3 is off, switch 4 is dont care
// Modes:
//  0 = 000 = Garage Mode, button light test to see if car alive!!
//             PUSHBUTTON A - Light LEDs 0 and 1
//             PUSHBUTTON B - Light LEDs 2 and 3
//            -switch 4 does nothing-
//
//  1 = 001 = Garage Mode, forward/reverse adjust, no auto steer, terminal output
//             POT1 = left motor speed adjust
//             POT0 = right motor speed adjust
//            -switch 4 does nothing-
//
//  2 = 010 = Garage Mode, steering adjust, no auto steer, terminal output
//             POT 1 - Controls Servo:
//                CCW = turn left
//                CW = turn right
//            -switch 4 does nothing-
//
//  3 = 011 = Garage Mode, Camera test, some auto steer, terminal output
//            switch 4:
//               OFF = normal dec data
//                ON = o-scope mode
//
//  4 = 100 = Track Mode, Auto Steer, safe settings
//            switch 4 = terminal output on/off (causes 2 second lag)
//
//  5 = 101 = Track Mode, Auto Steer, fast settings
//            switch 4 = terminal output on/off (causes 2 second lag)
//
//  6 = 110 = future upgrades
//
//  7 = 111 = future upgrades

/* NOTES
Camera unmounted
motors unhooked
only servo / steering hooked up 

exercise 1 - garage drive
- manual motors forward / reverse
- ensure motors hooked up properly - forward/reverse/left/right
- show terminal value

exercise 2 - garage steer
- manual steering - left /right 
- calibrate steering
- show terminal to get feedback

exercise 3 - garage see
- focus camera
- mount camera
- camera + servo line tracking (race mode)
garage mode use paper show following
- bad Kp value
- min speed = 0.5

exercise 4 - track - slow
- fine tuning max speed
- fine tune proportional control - Kp

exercise 5 - track - fast
- ratio of differential motor speed on curves
- dead zone with high speed on straights

*/

void TFC_TickerUpdate() {
    int i;

    for(i = 0; i < NUM_TFC_TICKERS; i++) {
        if(TFC_Ticker[i]<0xFFFFFFFF) {
            TFC_Ticker[i]++;
        }
    }
}

// Garage Mode
//   Car not meant to run on track
//   Use this to test out car features
//   and calibrate car
//
void GarageMode() {
    uint32_t i, j = 0;
    float ReadPot0, ReadPot1;

    // This Demo program will look at the first 2 switches to select one of 4 demo / garage modes
    switch(TFC_GetDIP_Switch() & 0x03) {
        default:
        case 0: // Mode 0
            TFC_HBRIDGE_DISABLE;
            TERMINAL_PRINTF("MODE 0\r\n");  
            
            if(TFC_PUSH_BUTTON_0_PRESSED) {
                TFC_BAT_LED0_ON;
                TFC_BAT_LED1_ON;
            } 
            else {
                TFC_BAT_LED0_OFF;
                TFC_BAT_LED1_OFF;
            }
            
            if(TFC_PUSH_BUTTON_1_PRESSED) {
                TFC_BAT_LED2_ON;
                TFC_BAT_LED3_ON;
            } 
            else {
                TFC_BAT_LED2_OFF;
                TFC_BAT_LED3_OFF;
            }     
            break;
                
        case 1: // Mode 1
            TFC_HBRIDGE_ENABLE;
            ReadPot0 = TFC_ReadPot(0);
            ReadPot1 = TFC_ReadPot(1);
            TERMINAL_PRINTF("MODE 1: ");   
            TERMINAL_PRINTF("Left drive setting = %1.2f\t\t", ReadPot1);
            TERMINAL_PRINTF("Right drive setting = %1.2f\r\n", ReadPot0);
            TFC_SetMotorPWM(ReadPot0, ReadPot1);  
            break;
        
        case 2:                  
            //Make sure motors are off 
            TFC_SetMotorPWM(0, 0);
            TFC_HBRIDGE_DISABLE;

            if(TFC_Ticker[0] >= 20) {// every 40mS output data to terminal
                TFC_Ticker[0] = 0; //reset the Ticker
                //update the Servos
                TERMINAL_PRINTF("MODE 2: ");   
                ReadPot0 = TFC_ReadPot(0);
                //ReadPot1 = TFC_ReadPot(1);
                TFC_SetServo(0, ReadPot0);
                //TFC_SetServo(1, ReadPot1);
                TERMINAL_PRINTF("Steer1 setting = %1.2f\r\n", ReadPot0);
                //TERMINAL_PRINTF("Steer2 setting = %1.2f\r\n", ReadPot0);
            }    
            break;
            
        case 3 :
            TFC_HBRIDGE_DISABLE;
            
            if(TFC_Ticker[0] > 1000 && TFC_LineScanImageReady > 0) {// every 1000 ticks (1s if 10ms)
                // TERMINAL_PRINTF("MODE 3:\r\n");
                TFC_Ticker[0] = 0;
                TFC_LineScanImageReady=0; // must reset to 0 after detecting non-zero

                if(terminalMode()) {
                    // print values to terminal as if were o-scope...
                    for(j = 20; j > 0; j--) {
                        for(i = 0; i < 128; i++) {
                            if ((TFC_LineScanImage0[i] <= (4096 * j / 20)) && (TFC_LineScanImage0[i] >= (4096 * (j - 1) / 20))) {
                                TERMINAL_PRINTF("*");
                            }
                            else {
                                TERMINAL_PRINTF(" ");
                            }
                        }
                        TERMINAL_PRINTF("\r\n");
                    }
                } 
                else { 
                    for(i = 0; i < 128; i++) {// print one line worth of data (128) from Camera 0
                        TERMINAL_PRINTF("%d", TFC_LineScanImage0[i]);
                      
                        if(i == 127) { // when last data reached put in line return
                            TERMINAL_PRINTF("\r\n", TFC_LineScanImage0[i]);
                        }
                        else {
                            TERMINAL_PRINTF(",", TFC_LineScanImage0[i]);               
                        }
                    }
                    TERMINAL_PRINTF("============================================================================================\r\n");
                    wait_ms(10);
                }       
            }
            break;
    }
}

int main() {
    PC.baud(115200); // Excel and TeraTerm 
    //PC.baud(9600); // USB
    TFC_TickerObj.attach_us(&TFC_TickerUpdate, 2000); // update ticker array every 2mS (2000 uS)
    TFC_Init();
    
    while(1) {   
        if(getMode() < 4) {
          // Run Garage Mode
          GarageMode();
        }
        else {     
          // Run Track Mode
          TrackMode();
        } 
    }
}
