
#include "mbed.h"

/** @file test.h*/

/**
 * @defgroup FRDM-TFC_API FRDM-TFC_API
 *
 * @{
 */


/**

@addtogroup FRDM-TFC_API

@{

Resources used by the TFC Library\n

I/O:\n
-------------------------------------------------------------------------------------------------\n

    PTB0   (Servo Channel 0 - TPM1)\n
    PTB1   (Servo Channel 1 - TPM1)\n
\n
    PTB8   (Battery LED0)\n
    PTB9   (Battery LED1)\n
    PTB10  (Battery LED2)\n
    PTB11  (Battery LED3)\n
\n
    PTD7   (Camera SI)\n
    PTE0   (Camera CLK)\n
    PTD5   (Camera A0  - ADC_SE6b)\n
    PTD6   (Camera A1 - ADC_SE7b)\n
\n
    PTE2    DIP Switch 0\n
    PTE3    DIP Switch 1\n
    PTE4    DIP Switch 2\n
    PTE5    DIP Switch 3\n

    PTC13   Pushbutton SW1\n
    PTC17   Pushbutton SW2\n

    PTC3    H-Bridge A - 1 FTM0_CH3\n
    PTC4    H-Bridge A - 2 FTM0_CH4\n
    PTC1    H-Bridge B - 1 FTM0_CH1\n
    PTC2    H-Bridge B - 2 FTM0_CH2\n

    PTE21   H-Bridge Enable\n
    PTE20   H-Bridge Fault\n

    PTE23   H-Bridge A - IFB\n
    PTE22   H-Bridge B - IFB\n

    }
*/



#ifndef _TFC_H
#define _TFC_H

#define TFC_HBRIDGE_EN_LOC          (uint32_t)(1<<21)
#define TFC_HBRIDGE_FAULT_LOC       (uint32_t)(1<<20)

#define TFC_HBRIDGE_ENABLE          PTE->PSOR = TFC_HBRIDGE_EN_LOC
#define TFC_HBRIDGE_DISABLE         PTE->PCOR = TFC_HBRIDGE_EN_LOC

#define TFC_DIP_SWITCH0_LOC         ((uint32_t)(1<<2))
#define TFC_DIP_SWITCH1_LOC         ((uint32_t)(1<<3))
#define TFC_DIP_SWITCH2_LOC         ((uint32_t)(1<<4))
#define TFC_DIP_SWITCH3_LOC         ((uint32_t)(1<<5))

#define TFC_PUSH_BUTT0N0_LOC        ((uint32_t)(1<<13))
#define TFC_PUSH_BUTT0N1_LOC        ((uint32_t)(1<<17))

#define TFC_BAT_LED0_LOC            ((uint32_t)(1<<11))
#define TFC_BAT_LED1_LOC            ((uint32_t)(1<<10))
#define TFC_BAT_LED2_LOC            ((uint32_t)(1<<9))
#define TFC_BAT_LED3_LOC            ((uint32_t)(1<<8))

#define TAOS_CLK_HIGH  PTE->PSOR = (1<<1)
#define TAOS_CLK_LOW   PTE->PCOR = (1<<1)
#define TAOS_SI_HIGH   PTD->PSOR = (1<<7)
#define TAOS_SI_LOW    PTD->PCOR = (1<<7)


/**

@addtogroup FRDM-TFC_API
@{
*/

/**Macro to turn on LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_ON             PTB->PSOR = TFC_BAT_LED0_LOC
/** Macro to turn on LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_ON             PTB->PSOR = TFC_BAT_LED1_LOC
/** Macro to turn on LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_ON             PTB->PSOR = TFC_BAT_LED2_LOC
/** Macro to turn on LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_ON             PTB->PSOR = TFC_BAT_LED3_LOC


/** Macro to turn off LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_OFF            PTB->PCOR = TFC_BAT_LED0_LOC
/** Macro to turn off LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_OFF            PTB->PCOR = TFC_BAT_LED1_LOC
/** Macro to turn off LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_OFF            PTB->PCOR = TFC_BAT_LED2_LOC
/** Macro to turn off LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_OFF            PTB->PCOR = TFC_BAT_LED3_LOC


/** Macro to toggle LED 0 in the battery indicator array*/
#define TFC_BAT_LED0_TOGGLE         PTB->PTOR = TFC_BAT_LED0_LOC
/** Macro to toggle LED 1 in the battery indicator array*/
#define TFC_BAT_LED1_TOGGLE         PTB->PTOR = TFC_BAT_LED1_LOC
/** Macro to toggle LED 2 in the battery indicator array*/
#define TFC_BAT_LED2_TOGGLE         PTB->PTOR = TFC_BAT_LED2_LOC
/** Macro to toggle LED 3 in the battery indicator array*/
#define TFC_BAT_LED3_TOGGLE         PTB->PTOR = TFC_BAT_LED3_LOC


/** Macro to read the state of the pushbutton SW1*/
#define TFC_PUSH_BUTTON_0_PRESSED   ((PTC->PDIR&TFC_PUSH_BUTT0N0_LOC)>0)
/** Macro to read the state of the pushbutton SW1*/
#define TFC_PUSH_BUTTON_1_PRESSED   ((PTC->PDIR&TFC_PUSH_BUTT0N1_LOC)>0)

/** Macro to read the state of switch 0 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_0_ON         ((TFC_GetDIP_Switch()&0x01)>0)

/** Macro to read the state of switch 1 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_1_ON         ((TFC_GetDIP_Switch()&0x02)>0)

/** Macro to read the state of switch 2 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_2_ON         ((TFC_GetDIP_Switch()&0x04)>0)

/** Macro to read the state of switch 3 in the 4 position DIP switch*/
#define TFC_DIP_SWITCH_3_ON         ((TFC_GetDIP_Switch()&0x08)>0)


/** Initialized the TFC API.   Call before using any other API calls.
*
*/
void TFC_Init();

/** ServoTicker will increment once every servo cycle.
* It can be used to synchronize events to the start of a servo cycle. ServoTicker is a volatile uint32_t and is updated in the TPM1 overlflow interrupt.  This means you will see ServoTicker increment on the rising edge of the servo PWM signal
*
*/
 extern volatile uint32_t TFC_ServoTicker;


/** Gets the state of the 4-positiomn DIP switch on the FRDM-TFC
*
*  @returns The lower 4-bits of the return value map to the 4-bits of the DIP switch
*/
uint8_t TFC_GetDIP_Switch();


/** Reads the state of the pushbuttons (SW1, SW2)  on the FRDM-TFC
*  @param Index Selects the pushbutton (0 for SW1 and 1 for SW2)
*  @returns A non-zero value if the button is pushed
*/
uint8_t TFC_ReadPushButton(uint8_t Index);


/** Controls the 4 battery level LEDs on the FRDM-TFC boards.
*
*  @param Value  The lower 4-bits of the parameter maps to the 4 LEDs.
*/
void TFC_SetBatteryLED(uint8_t Value);


/** Sets the servo channels
*
*  @param ServoNumber  Which servo channel on the FRDM-TFC to use (0 or 1).  0 is the default channel for steering
*  @param Position     Angle setting for servo in a normalized (-1.0 to 1.0) form.   The range of the servo can be changed with the InitServos function.
*                       This is called in the TFC constructor with some useful default values-->  20mSec period,  0.5mS min and 2.0mSec max.  you may need to adjust these for your own particular setup.
*/
void TFC_SetServo(uint8_t ServoNumber, float Position);

/** Initializes TPM for the servoes.  It also sets the max and min ranges
*
*  @param ServoPulseWidthMin    Minimum pulse width (in seconds) for the servo.   The value of -1.0 in SetServo is mapped to this pulse width.  I.E.  .001
*  @param ServoPulseWidthMax    Maximum pulse width (in seconds) for the servo.   The value of +1.0 in SetServo is mapped to this pulse width.  I.E.  .002
*  @param ServoPeriod           Period of the servo pulses (in seconds).  I.e.  .020 for 20mSec
*/

void TFC_InitServos(float ServoPulseWidthMin, float ServoPulseWidthMax, float ServoPeriod);


/** Initialized TPM0 to be used for generating PWM signals for the the dual drive motors.   This method is called in the TFC constructor with a default value of 4000.0Hz
*
*  @param SwitchingFrequency PWM Switching Frequency in floating point format.   Pick something between 1000 and 9000.   Maybe you can modulate it and make a tune.
*/
void TFC_InitMotorPWM(float SwitchingFrequency);

/** Sets the PWM value for each motor.
*
*  @param MotorA    The PWM value for HBridgeA. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*  @param MotorB    The PWM value for HBridgeB. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*/
void TFC_SetMotorPWM(float MotorA ,float MotorB);

/** Reads the potentiometers
*
*  @param Channel   Selects which pot is read.   I.e.  0 for POT0 or 1 for POT1
*  @returns    Pot value from -1.0 to 1.0
*/
float TFC_ReadPot(uint8_t Channel);

/** Gets the current battery voltage
*
*  @returns    Battery voltage in floating point form.
*/
float TFC_ReadBatteryVoltage();



/** Sets the Battery level indiciate
*
*  @param BattLevel   A number betwween 0 and 4.   This will light the bar from left to right with the specifified number of segments.
*
*/
void TFC_SetBatteryLED_Level(uint8_t BattLevel);


/** Pointer to two channels of line scan camera data.   Each channel is 128 points of uint8_t's.  Note that the underlying implementation is ping-pong buffer  These pointers will point to the 
*inactive buffer.   
*
*/

extern volatile uint16_t * TFC_LineScanImage0;
extern volatile uint16_t * TFC_LineScanImage1;


/** This flag will increment when a new frame is ready.  Check for a non zero value (and reset to zero!) when you want to read the camera(s)
*
*/

extern volatile uint8_t TFC_LineScanImageReady;





/** @} */


#endif
