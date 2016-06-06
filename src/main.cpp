#include "mbed.h"
#include "TFC.h"
#include "calibrate.h"
#include "common.h"
#include "engine.h"

float PROTO_MAX_SPEED = 0.0;

void TFC_TickerUpdate() {
    int i;

    for(i = 0; i < NUM_TFC_TICKERS; i++) {
        if(TFC_Ticker[i] < 0xFFFFFFFF) {
            TFC_Ticker[i]++;
        }
    }
}

 /****************** CalibrateSwitches() ****************** 
 *
 * Press button 0 or 1 and toggles
 * LED 0 1 and 2 3 respectively.
 * 
 **********************************************************/
void CalibrateSwitches() {
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
}

 /****************** CalibrateServo() ****************** 
 *
 * - Disables the HBridge
 * Read from the pot and adjust the servo accoding to
 * that value.
 * 
 *******************************************************/
void CalibrateServo() {
    float potValue0, potValue1;

    TFC_HBRIDGE_DISABLE;            
    if(TFC_Ticker[0] >= 20) { // every 40ms...
        TFC_Ticker[0] = 0;    //reset the Ticker
        
        //update the Servos
        potValue0 = TFC_ReadPot(0);
        potValue1 = TFC_ReadPot(1);
        TFC_SetServo(0, potValue0);
        TFC_SetServo(1, potValue1);
        TERMINAL_PRINTF("Pot0 = %1.2f\r\n", potValue0);
        TERMINAL_PRINTF("Pot1 = %1.2f\r\n", potValue1);
    }

    TFC_SetBatteryLED_Level(2); // Battery level 2 indicates servo calibration
    TFC_SetMotorPWM(0, 0);      // Make sure motors are off 
    TFC_HBRIDGE_DISABLE;
}

 /****************** CalibrateMotors() ****************** 
 *
 * - Enables the HBridge
 * Read from the two pots and adjust either left or right
 * motor speed depending on the two pot values.
 * 
 *******************************************************/
void CalibrateMotors() {
    float potValue0, potValue1;

    TFC_HBRIDGE_ENABLE;
    potValue0 = TFC_ReadPot(0);
    potValue1 = TFC_ReadPot(1);
    TERMINAL_PRINTF("Pot0 = %1.2f\n", potValue0);
    TERMINAL_PRINTF("Pot1 = %1.2f\n", potValue1);
    TFC_SetMotorPWM(potValue0, potValue1);
    TFC_SetBatteryLED_Level(3); // Battery level 3 indicates servo calibration
}

 /****************** CalibrateCamera() ****************** 
 *
 * Grabs ADC values from the camera and prints to Screen
 * 
 *******************************************************/
void CalibrateCamera() {
    uint32_t i, j, t = 0;

    if(TFC_Ticker[0] > 1000 && TFC_LineScanImageReady > 0) { // every 2s ...
        TFC_Ticker[0] = 0;
        TFC_LineScanImageReady = 0; // must reset to 0 after detecting non-zero
        TFC_SetBatteryLED_Level(4);
        for(i = 0; i < 8; i++) { // print one line worth of data (128) from Camera 0
            for(j = 0; j < 16; j++) {
                    TERMINAL_PRINTF("0x%X", TFC_LineScanImage0[(i * 16) + j]);
                    if((i == 7) && (j == 15)) {  // when last data reached put in line return
                       TERMINAL_PRINTF("\r\n",TFC_LineScanImage0[(i * 16) + j]);
                    }
                    else {
                       TERMINAL_PRINTF(", ",TFC_LineScanImage0[(i * 16) + j]);
                    }
            }
            wait_ms(50);
        } 
    }
}

/****************** ExecuteCalibration() ****************** 
 *
 * default: No default behavior defined
 * case 0: Switches - No dip switch toggled high
 * case 1: Servo - Dip switch 2 toggled high
 * case 2: Motor/PWM - Dip switch 3 toggled high
 * case 3: Camera - Dip switches 2 and 3 are toggled high
 * 
 **********************************************************/
void ExecuteCalibration() {

    switch((TFC_GetDIP_Switch() >> 1) & 0x03) {
        default:

        case 0 :
            CalibrateSwitches();
            break;
        
        case 1:
            CalibrateServo();
            break;
         
        case 2 :
            CalibrateMotors();
            break;
     
        case 3:
            CalibrateCamera();
            break;
    }
}

void SetMaxSpeed() {
    PROTO_MAX_SPEED = TFC_ReadPot(0);
}

void ExecutePrototype() {
    if(TFC_PUSH_BUTTON_0_PRESSED) {
        TFC_BAT_LED0_ON;
        TFC_BAT_LED1_ON;
        TFC_BAT_LED2_ON;
        TFC_BAT_LED3_ON;
        SetMaxSpeed();

        TFC_HBRIDGE_ENABLE;
        TFC_SetMotorPWM(PROTO_MAX_SPEED, PROTO_MAX_SPEED);
    }

    if(TFC_PUSH_BUTTON_1_PRESSED) {
        TFC_BAT_LED0_OFF;
        TFC_BAT_LED1_OFF;
        TFC_BAT_LED2_OFF;
        TFC_BAT_LED3_OFF;

        TFC_SetMotorPWM(0, 0);
        TFC_HBRIDGE_DISABLE;
    }
}

int main() {
    //PC.baud(115200); // Excel and TeraTerm 
    PC.baud(9600);     // UNIX - screen /var/tty.usbmodem*
    TFC_TickerObj.attach_us(&TFC_TickerUpdate, 2000); // update ticker array every 2mS (2000 uS)
    TFC_Init();
    
    while(1) {      
        //TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
        //TFC_Task();

        // If DIP switch 1 is high, then run MCP, else Demo program
        if(TFC_GetDIP_Switch() & 0x01) {
            //ExecuteEngine();
            ExecutePrototype();
        }
        else {     
            ExecuteCalibration();
 		}
    }
}
