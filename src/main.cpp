#include "mbed.h"
#include "TFC.h"
#include "common.h"
#include "Spices.h"

void TFC_TickerUpdate() {
    int i;
 
    for(i = 0; i < NUM_TFC_TICKERS; i++) {
        if(TFC_Ticker[i] < 0xFFFFFFFF) {
            TFC_Ticker[i]++;
        }
    }
}

void DemoProgram() {
    uint32_t i, j, t = 0;
    float ReadPot0, ReadPot1;

    //This Demo program will look at the middle 2 switch to select one of 4 demo modes.
    //Let's look at the middle 2 switches
    switch((TFC_GetDIP_Switch() >> 1) & 0x03) {
        default:
        case 0 :
            //Demo mode 0 just tests the switches and LED's
            if(TFC_PUSH_BUTTON_0_PRESSED) {
                TFC_BAT_LED0_ON;
            }
            else {
                TFC_BAT_LED0_OFF;
            }
            if(TFC_PUSH_BUTTON_1_PRESSED) {
                TFC_BAT_LED3_ON;
            }
            else {
                TFC_BAT_LED3_OFF;
            }
            /*if(TFC_GetDIP_Switch()&0x01)
                TFC_BAT_LED1_ON;
            else
                TFC_BAT_LED1_OFF;
            if(TFC_GetDIP_Switch()&0x08)
                TFC_BAT_LED2_ON;
            else
                TFC_BAT_LED2_OFF; */
            break;
              
        case 1:
            TFC_HBRIDGE_DISABLE;            
            //if (TFC_HBRIDGE_ENABLED) {  
                //TFC_HBRIDGE_ENABLED = false;
            //}

            //Demo mode 1 will just move the servos with the on-board potentiometers
            if(TFC_Ticker[0] >= 20) { // every 40mS...
                TFC_Ticker[0] = 0; //reset the Ticker
                
                //update the Servos
                ReadPot0 = TFC_ReadPot(0);
                ReadPot1 = TFC_ReadPot(1);
                TFC_SetServo(0,ReadPot0);
                TFC_SetServo(1,ReadPot1);
                //TERMINAL_PRINTF("Pot0 = %1.2f\r\n", ReadPot0);
                //TERMINAL_PRINTF("Pot1 = %1.2f\r\n", ReadPot1);
            }
            
            //Let's put a pattern on the LEDs
            if(TFC_Ticker[1] >= 125) { // every 250mS... cycle through LEDs
              TFC_Ticker[1] = 0;
              t++;
              if(t > 4) {
                  t = 0;
              }           
              TFC_SetBatteryLED_Level(t);
            }
            
            TFC_SetMotorPWM(0, 0); //Make sure motors are off 
            TFC_HBRIDGE_DISABLE;
            
            break;
          
        case 2 :
            TFC_HBRIDGE_ENABLE;
            ReadPot0 = TFC_ReadPot(0);
            ReadPot1 = TFC_ReadPot(1);
            //TERMINAL_PRINTF("Pot0 = %1.2f\n", ReadPot0);
            //TERMINAL_PRINTF("Pot1 = %1.2f\n", ReadPot1);
            TFC_SetMotorPWM(ReadPot0,ReadPot1);
                  
            //Let's put a pattern on the LEDs
            if(TFC_Ticker[1] >= 125) {
                TFC_Ticker[1] = 0;
                t++;
                
                if(t > 4) {
                    t = 0;
                }           
                TFC_SetBatteryLED_Level(t);
            }
            break;
      
        case 3:
            //Demo Mode 3 will be in Freescale Garage Mode.  It will beam data from the Camera to the 
            //Labview Application
            //note that there are some issues 
            if(TFC_Ticker[0] > 1000 && TFC_LineScanImageReady > 0) { // every 2s ...
                TFC_Ticker[0] = 0;
                TFC_LineScanImageReady=0; // must reset to 0 after detecting non-zero
              
                if(t == 0) {
                      t = 4;
                }
                else {
                    t--;
                }
                TFC_SetBatteryLED_Level(t);
                for(i = 0; i < 8; i++) { // print one line worth of data (128) from Camera 0
                    for(j = 0; j < 16; j++) {
                            TERMINAL_PRINTF("0x%X",TFC_LineScanImage0[(i*16)+j]);
                            if((i == 7) && (j == 15)) {  // when last data reached put in line return
                               TERMINAL_PRINTF("\r\n",TFC_LineScanImage0[(i*16)+j]);
                            }
                            else {
                               TERMINAL_PRINTF(",",TFC_LineScanImage0[(i*16)+j]);
                            }
                    }
                    wait_ms(10);
                }
                  
                /*for(i=0;i<8;i++) { // print one line worth of data (128) from Camera 1 ??
                    for(j=0;j<16;j++) {
    	                TERMINAL_PRINTF("0x%X",TFC_LineScanImage1[(i*16)+j]);
    	              
    	                if((i==7) && (j==15))  // when last data reached put in line return
    	                    TERMINAL_PRINTF("\r\n",TFC_LineScanImage1[(i*16)+j]);
    	                else
    	                    TERMINAL_PRINTF(",",TFC_LineScanImage1[(i*16)+j]);
    	            }         
                    wait_ms(10);  
                }*/        
            }
        break;
  } // end case
}

 
int main() {
    // TERMINAL TYPE  
    PC.baud(115200); // works with Excel and TeraTerm 
    //PC.baud(9600); // works with USB Serial Monitor Lite: https://play.google.com/store/apps/details?id=jp.ksksue.app.terminal; doesn't work > 9600
    TFC_TickerObj.attach_us(&TFC_TickerUpdate, 2000); // update ticker array every 2mS (2000 uS)
   
    TFC_Init();
    
    for(;;) {      
        //TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
        //TFC_Task();

        // If DIP switch 1 is high, then run MCP, else Demo program
        if(TFC_GetDIP_Switch() & 0x01) {
            // Run MCP
            MasterControlProgram();
        }
        else {     
            // Run Demo Program
            DemoProgram();
 		}
    } // end of infinite for loop
}
