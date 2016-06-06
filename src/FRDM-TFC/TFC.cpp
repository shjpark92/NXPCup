#include "mbed.h"
#include "TFC.h"

#define FTM1_CLK_PRESCALE                                                                  6   // Prescale Selector value - see comments in Status Control (SC) section for more details
#define SERVO_DEFAULT_PERIOD                                                   (float)(.010)   // Desired Frequency of PWM Signal - Here 50Hz => 20ms period
#define TAOS_CLK_COUNT                                                                   200   // Number of cycles for CLK Signal on camera

// use these to dial in servo steering to your particular servo
#define SERVO_MIN_PULSE_WIDTH_DEFAULT                                          (float)(.0005)  // The number here should be be *pulse width* in seconds to move servo to its left limit
#define SERVO_MAX_PULSE_WIDTH_DEFAULT                                          (float)(.002)   // The number here should be be *pulse width* in seconds to move servo to its left limit 


#define FTM0_CLOCK                                             (SystemCoreClock/2)
#define FTM0_CLK_PRESCALE                                      (0)  // Prescale Selector value - see comments in Status Control (SC) section for more details
#define FTM0_DEFAULT_SWITCHING_FREQUENCY                      (4000.0)

#define ADC_MAX_CODE    (4095)

#define TAOS_CLK_HIGH  PTE->PSOR = (1<<1)
#define TAOS_CLK_LOW   PTE->PCOR = (1<<1)
#define TAOS_SI_HIGH   PTD->PSOR = (1<<7)
#define TAOS_SI_LOW    PTD->PCOR = (1<<7)

#define ADC_STATE_INIT                          0
#define ADC_STATE_CAPTURE_POT_0                 1
#define ADC_STATE_CAPTURE_POT_1                 2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL         3
#define ADC_STATE_CAPTURE_LINE_SCAN             4


#define TFC_POT_0_ADC_CHANNEL       13
#define TFC_POT_1_ADC_CHANNEL       12
#define TFC_BAT_SENSE_CHANNEL       4
#define TFC_LINESCAN0_ADC_CHANNEL   6
#define TFC_LINESCAN1_ADC_CHANNEL   7


#define ADC0_irq_no   57
#define ADC1_irq_no   58

#define ADC0_CHANA    19                                    // set to desired ADC0 channel trigger A    
#define ADC0_CHANB    20                                    // set to desired ADC0 channel trigger B    

#define ADC1_CHANA    20                                    // set to desired ADC1 channel trigger A  20 defaults to potentiometer in TWRK60     
#define ADC1_CHANB    20                                    // set to desired ADC1 channel trigger B

#define ADC0_DLYA     0x2000                                // ADC0 trigger A delay 
#define ADC0_DLYB     0x4000                                // ADC0 trigger B delay 
#define ADC1_DLYA     0x6000                                // ADC1 trigger A delay
#define ADC1_DLYB     0x7fff                                // ADC1 trigger B delay 


#define ADC0A_DONE   0x01
#define ADC0B_DONE   0x02
#define ADC1A_DONE   0x04
#define ADC1B_DONE   0x08


// Bit shifting of bitfiled is already taken into account so
// bitfiled values are always represented as relative to their position.

/************************* #Defines ******************************************/

#define A                 0x0
#define B                 0x1

/////// NOTE: the following defines relate to the ADC register definitions
/////// and the content follows the reference manual, using the same symbols.


//// ADCSC1 (register)

// Conversion Complete (COCO) mask
#define COCO_COMPLETE     ADC_SC1_COCO_MASK
#define COCO_NOT          0x00

// ADC interrupts: enabled, or disabled.
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

// Differential or Single ended ADC input
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

//// ADCCFG1

// Power setting of ADC
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

// Clock divisor
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

// Long samle time, or Short sample time
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

// How many bits for the conversion?  8, 12, 10, or 16 (single ended).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03



// ADC Input Clock Source choice? Bus clock, Bus clock/2, "altclk", or the
//                                ADC's own asynchronous clock for less noise
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

//// ADCCFG2

// Select between B or A channels
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

// Ansync clock output enable: enable, or disable the output of it
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

// High speed or low speed conversion mode
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

// Long Sample Time selector: 20, 12, 6, or 2 extra clocks for a longer sample time
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

////ADCSC2

// Read-only status bit indicating conversion status
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00

// Trigger for starting conversion: Hardware trigger, or software trigger.
// For using PDB, the Hardware trigger option is selected.
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

// Voltage Reference selection for the ADC conversions
// (***not*** the PGA which uses VREFO only).
// VREFH and VREFL (0) , or VREFO (1).

#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     /* reserved */
#define REFSEL_RES_EXT     0x03     /* reserved but defaults to Vref */

////ADCSC3

// Calibration begin or off
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00

// Status indicating Calibration failed, or normal success
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00

// ADC to continously convert, or do a sinle conversion
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

// Averaging enabled in the ADC, or not.
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00

// How many to average prior to "interrupting" the MCU?  4, 8, 16, or 32
#define AVGS_4             0x00
#define AVGS_8             0x01
#define AVGS_16            0x02
#define AVGS_32            0x03

////PGA

// PGA enabled or not?
#define PGAEN_ENABLED      ADC_PGA_PGAEN_MASK
#define PGAEN_DISABLED     0x00

// Chopper stabilization of the amplifier, or not.
#define PGACHP_CHOP        ADC_PGA_PGACHP_MASK
#define PGACHP_NOCHOP      0x00

// PGA in low power mode, or normal mode.
#define PGALP_LOW          ADC_PGA_PGALP_MASK
#define PGALP_NORMAL       0x00

// Gain of PGA.  Selectable from 1 to 64.
#define PGAG_1             0x00
#define PGAG_2             0x01
#define PGAG_4             0x02
#define PGAG_8             0x03
#define PGAG_16            0x04
#define PGAG_32            0x05
#define PGAG_64            0x06


#define ADC_STATE_INIT                            0
#define ADC_STATE_CAPTURE_POT_0                   1
#define ADC_STATE_CAPTURE_POT_1                   2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL           3
#define ADC_STATE_CAPTURE_LINE_SCAN               4


/////////// The above values fit into the structure below to select ADC/PGA
/////////// configuration desired:

typedef struct adc_cfg {
    uint8_t  CONFIG1;
    uint8_t  CONFIG2;
    uint16_t COMPARE1;
    uint16_t COMPARE2;
    uint8_t  STATUS2;
    uint8_t  STATUS3;
    uint8_t  STATUS1A;
    uint8_t  STATUS1B;
    uint32_t PGA;
} *tADC_ConfigPtr, tADC_Config ;


#define CAL_BLK_NUMREC 18

typedef struct adc_cal {

    uint16_t  OFS;
    uint16_t  PG;
    uint16_t  MG;
    uint8_t   CLPD;
    uint8_t   CLPS;
    uint16_t  CLP4;
    uint16_t  CLP3;
    uint8_t   CLP2;
    uint8_t   CLP1;
    uint8_t   CLP0;
    uint8_t   dummy;
    uint8_t   CLMD;
    uint8_t   CLMS;
    uint16_t  CLM4;
    uint16_t  CLM3;
    uint8_t   CLM2;
    uint8_t   CLM1;
    uint8_t   CLM0;
} tADC_Cal_Blk ;

typedef struct ADC_MemMap {
    uint32_t SC1[2];                                 /**< ADC Status and Control Registers 1, array offset: 0x0, array step: 0x4 */
    uint32_t CFG1;                                   /**< ADC Configuration Register 1, offset: 0x8 */
    uint32_t CFG2;                                   /**< ADC Configuration Register 2, offset: 0xC */
    uint32_t R[2];                                   /**< ADC Data Result Register, array offset: 0x10, array step: 0x4 */
    uint32_t CV1;                                    /**< Compare Value Registers, offset: 0x18 */
    uint32_t CV2;                                    /**< Compare Value Registers, offset: 0x1C */
    uint32_t SC2;                                    /**< Status and Control Register 2, offset: 0x20 */
    uint32_t SC3;                                    /**< Status and Control Register 3, offset: 0x24 */
    uint32_t OFS;                                    /**< ADC Offset Correction Register, offset: 0x28 */
    uint32_t PG;                                     /**< ADC Plus-Side Gain Register, offset: 0x2C */
    uint32_t MG;                                     /**< ADC Minus-Side Gain Register, offset: 0x30 */
    uint32_t CLPD;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x34 */
    uint32_t CLPS;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x38 */
    uint32_t CLP4;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x3C */
    uint32_t CLP3;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x40 */
    uint32_t CLP2;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x44 */
    uint32_t CLP1;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x48 */
    uint32_t CLP0;                                   /**< ADC Plus-Side General Calibration Value Register, offset: 0x4C */
    uint8_t RESERVED_0[4];
    uint32_t CLMD;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x54 */
    uint32_t CLMS;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x58 */
    uint32_t CLM4;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x5C */
    uint32_t CLM3;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x60 */
    uint32_t CLM2;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x64 */
    uint32_t CLM1;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x68 */
    uint32_t CLM0;                                   /**< ADC Minus-Side General Calibration Value Register, offset: 0x6C */
} volatile *ADC_MemMapPtr;



/* ADC - Register accessors */
#define ADC_SC1_REG(base,index)                  ((base)->SC1[index])
#define ADC_CFG1_REG(base)                       ((base)->CFG1)
#define ADC_CFG2_REG(base)                       ((base)->CFG2)
#define ADC_R_REG(base,index)                    ((base)->R[index])
#define ADC_CV1_REG(base)                        ((base)->CV1)
#define ADC_CV2_REG(base)                        ((base)->CV2)
#define ADC_SC2_REG(base)                        ((base)->SC2)
#define ADC_SC3_REG(base)                        ((base)->SC3)
#define ADC_OFS_REG(base)                        ((base)->OFS)
#define ADC_PG_REG(base)                         ((base)->PG)
#define ADC_MG_REG(base)                         ((base)->MG)
#define ADC_CLPD_REG(base)                       ((base)->CLPD)
#define ADC_CLPS_REG(base)                       ((base)->CLPS)
#define ADC_CLP4_REG(base)                       ((base)->CLP4)
#define ADC_CLP3_REG(base)                       ((base)->CLP3)
#define ADC_CLP2_REG(base)                       ((base)->CLP2)
#define ADC_CLP1_REG(base)                       ((base)->CLP1)
#define ADC_CLP0_REG(base)                       ((base)->CLP0)
#define ADC_CLMD_REG(base)                       ((base)->CLMD)
#define ADC_CLMS_REG(base)                       ((base)->CLMS)
#define ADC_CLM4_REG(base)                       ((base)->CLM4)
#define ADC_CLM3_REG(base)                       ((base)->CLM3)
#define ADC_CLM2_REG(base)                       ((base)->CLM2)
#define ADC_CLM1_REG(base)                       ((base)->CLM1)
#define ADC_CLM0_REG(base)                       ((base)->CLM0)

#define ADC0_BASE_PTR                            ((ADC_MemMapPtr)0x4003B000u)
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0_BASE_PTR }


float _ServoDutyCycleMin;
float _ServoDutyCycleMax;
float _ServoPeriod;

volatile uint16_t QueuedServo0Val;
volatile uint16_t QueuedServo1Val;

volatile uint16_t  *LineScanImage0WorkingBuffer;
volatile uint16_t  *LineScanImage1WorkingBuffer;

volatile uint16_t  LineScanImage0Buffer[2][128];
volatile uint16_t  LineScanImage1Buffer[2][128];
volatile uint8_t  LineScanWorkingBuffer;

volatile uint16_t * TFC_LineScanImage0;
volatile uint16_t * TFC_LineScanImage1;
volatile uint8_t  TFC_LineScanImageReady;

volatile uint16_t  PotADC_Value[2];
volatile uint16_t  BatSenseADC_Value;
volatile uint16_t  CurrentADC_State;
volatile uint8_t  CurrentLineScanPixel;
volatile uint8_t  CurrentLineScanChannel;
volatile uint32_t TFC_ServoTicker;


void TFC_SetServoDutyCycle(uint8_t ServoNumber, float DutyCycle);
void TFC_InitLineScanCamera();
uint8_t ADC_Cal(ADC_MemMapPtr adcmap);
void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr);
void ADC_Read_Cal(ADC_MemMapPtr adcmap, tADC_Cal_Blk *blk);
void TFC_InitADC0();
void TFC_InitADC_System();
void TFC_GPIO_Init();
void ADC0_Handler();
void TPM1_Handler();


void TFC_Init()
{

    TFC_GPIO_Init();

    TFC_InitADC_System(); // Always call this before the Servo init function....  The IRQ for the Servo code modifies ADC registers and the clocks need enable to the ADC peripherals 1st!

    TFC_InitLineScanCamera();

    TFC_InitServos(SERVO_MIN_PULSE_WIDTH_DEFAULT , SERVO_MAX_PULSE_WIDTH_DEFAULT, SERVO_DEFAULT_PERIOD);
    
    TFC_ServoTicker = 0;

    TFC_InitMotorPWM(FTM0_DEFAULT_SWITCHING_FREQUENCY);

}


void TFC_GPIO_Init()
{

    //enable Clocks to all ports

    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    //Setup Pins as GPIO
    PORTE->PCR[21] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    PORTE->PCR[20] = PORT_PCR_MUX(1);

    //Port for Pushbuttons
    PORTC->PCR[13] = PORT_PCR_MUX(1);
    PORTC->PCR[17] = PORT_PCR_MUX(1);


    //Ports for DIP Switches
    PORTE->PCR[2] = PORT_PCR_MUX(1);
    PORTE->PCR[3] = PORT_PCR_MUX(1);
    PORTE->PCR[4] = PORT_PCR_MUX(1);
    PORTE->PCR[5] = PORT_PCR_MUX(1);

    //Ports for LEDs
    PORTB->PCR[8] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    PORTB->PCR[9] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;


    //Setup the output pins
    PTE->PDDR =  TFC_HBRIDGE_EN_LOC;
    PTB->PDDR =  TFC_BAT_LED0_LOC  | TFC_BAT_LED1_LOC | TFC_BAT_LED2_LOC | TFC_BAT_LED3_LOC;

    TFC_SetBatteryLED(0);
    TFC_HBRIDGE_DISABLE;
}

void TFC_SetBatteryLED(uint8_t Value)
{
    if(Value & 0x01)
        TFC_BAT_LED0_ON;
    else
        TFC_BAT_LED0_OFF;

    if(Value & 0x02)
        TFC_BAT_LED1_ON;
    else
        TFC_BAT_LED1_OFF;

    if(Value & 0x04)
        TFC_BAT_LED2_ON;
    else
        TFC_BAT_LED2_OFF;

    if(Value & 0x08)
        TFC_BAT_LED3_ON;
    else
        TFC_BAT_LED3_OFF;
}

uint8_t TFC_GetDIP_Switch()
{
    uint8_t DIP_Val=0;

    DIP_Val = (PTE->PDIR>>2) & 0xF;

    return DIP_Val;
}

uint8_t TFC_ReadPushButton(uint8_t Index)
{
    if(Index == 0) {
        return TFC_PUSH_BUTTON_0_PRESSED;
    } else {
        return TFC_PUSH_BUTTON_1_PRESSED;
    }
}

extern "C" void TPM1_IRQHandler()
{
    //Clear the overflow mask if set.   According to the reference manual, we clear by writing a logic one!
    if(TPM1->SC & TPM_SC_TOF_MASK)
        TPM1->SC |= TPM_SC_TOF_MASK;

    //Dump the queued values to the timer channels
    TPM1->CONTROLS[0].CnV = QueuedServo0Val;
    TPM1->CONTROLS[1].CnV = QueuedServo1Val;


    //Prime the next ADC capture cycle
    TAOS_SI_HIGH;
    //Prime the ADC pump and start capturing POT 0
    CurrentADC_State = ADC_STATE_CAPTURE_POT_0;

    ADC0->CFG2  &= ~ADC_CFG2_MUXSEL_MASK; //Select the A side of the mux
    ADC0->SC1[0]  =  TFC_POT_0_ADC_CHANNEL | ADC_SC1_AIEN_MASK;  //Start the State machine at POT0

    //Flag that a new cervo cycle will start
    if (TFC_ServoTicker < 0xffffffff)//if servo tick less than max value, count up...
        TFC_ServoTicker++;

}


void TFC_InitServos(float PulseWidthMin, float PulseWidthMax, float ServoPeriod)
{

    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; 

    _ServoPeriod = ServoPeriod;
    _ServoDutyCycleMin = PulseWidthMin/ServoPeriod;
    _ServoDutyCycleMax = PulseWidthMax/ServoPeriod;

    //Clock Setup for the TPM requires a couple steps.
    SIM->SCGC6 &= ~SIM_SCGC6_TPM1_MASK;
    //1st,  set the clock mux
    //See Page 124 of f the KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual, Rev. 3, September 2012)
    SIM->SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    //Enable the Clock to the FTM0 Module
    //See Page 207 of f the KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    //The TPM Module has Clock.  Now set up the peripheral

    //Blow away the control registers to ensure that the counter is not running
    TPM1->SC = 0;
    TPM1->CONF = 0;

    //While the counter is disabled we can setup the prescaler

    TPM1->SC = TPM_SC_PS(FTM1_CLK_PRESCALE);
    TPM1->SC |= TPM_SC_TOIE_MASK; //Enable Interrupts for the Timer Overflow

    //Setup the mod register to get the correct PWM Period

    TPM1->MOD = (SystemCoreClock/(1<<(FTM1_CLK_PRESCALE))) * _ServoPeriod;
    //Setup Channels 0 and 1

    TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;


    //Set the Default duty cycle to servo neutral
    TFC_SetServo(0, 0.0);
    TFC_SetServo(1, 0.0);

    //Enable the TPM COunter
    TPM1->SC |= TPM_SC_CMOD(1);

    //Enable TPM1 IRQ on the NVIC

    //NVIC_SetVector(TPM1_IRQn,(uint32_t)TPM1_Handler);
    NVIC_EnableIRQ(TPM1_IRQn);

    //Enable the FTM functions on the the port

    PORTB->PCR[0] = PORT_PCR_MUX(3);
    PORTB->PCR[1] = PORT_PCR_MUX(3);

}


void TFC_SetServoDutyCycle(uint8_t ServoNumber, float DutyCycle)
{
    switch(ServoNumber) {
        default:
        case 0:

            QueuedServo0Val = TPM1->MOD * DutyCycle;

            break;

        case 1:

            QueuedServo1Val = TPM1->MOD * DutyCycle;

            break;
    }
}

void TFC_SetServo(uint8_t ServoNumber, float Position)
{
    TFC_SetServoDutyCycle(ServoNumber ,
                          ((Position + 1.0)/2)    *   ((_ServoDutyCycleMax - _ServoDutyCycleMin))+_ServoDutyCycleMin) ;

}

//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
//           _____   _____       ______ _    _ _   _  _____ _______ _____ ____  _   _  _____
//     /\   |  __ \ / ____|     |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
//    /  \  | |  | | |          | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___
//   / /\ \ | |  | | |          |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \
//  / ____ \| |__| | |____      | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
// /_/    \_\_____/ \_____|     |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/
// ********************************************************************************************************
// ********************************************************************************************************
// ********************************************************************************************************





uint8_t ADC_Cal(ADC_MemMapPtr adcmap)
{

    uint16_t cal_var;

    ADC_SC2_REG(adcmap) &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process    - ADC0_SC2 = ADC0_SC2 | ADC_SC2_ADTRGW(0);
    ADC_SC3_REG(adcmap) &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing
    ADC_SC3_REG(adcmap) |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(AVGS_32) );  // Turn averaging ON and set at max value ( 32 )


    ADC_SC3_REG(adcmap) |= ADC_SC3_CAL_MASK ;      // Start CAL
    while ( (ADC_SC1_REG(adcmap,A) & ADC_SC1_COCO_MASK ) == COCO_NOT ); // Wait calibration end

    if ((ADC_SC3_REG(adcmap)& ADC_SC3_CALF_MASK) == CALF_FAIL ) {
        return(1);    // Check for Calibration fail error and return
    }
    // Calculate plus-side calibration
    cal_var = 0x00;

    cal_var =  ADC_CLP0_REG(adcmap);
    cal_var += ADC_CLP1_REG(adcmap);
    cal_var += ADC_CLP2_REG(adcmap);
    cal_var += ADC_CLP3_REG(adcmap);
    cal_var += ADC_CLP4_REG(adcmap);
    cal_var += ADC_CLPS_REG(adcmap);

    cal_var = cal_var/2;
    cal_var |= 0x8000; // Set MSB

    ADC_PG_REG(adcmap) = ADC_PG_PG(cal_var);


    // Calculate minus-side calibration
    cal_var = 0x00;

    cal_var =  ADC_CLM0_REG(adcmap);
    cal_var += ADC_CLM1_REG(adcmap);
    cal_var += ADC_CLM2_REG(adcmap);
    cal_var += ADC_CLM3_REG(adcmap);
    cal_var += ADC_CLM4_REG(adcmap);
    cal_var += ADC_CLMS_REG(adcmap);

    cal_var = cal_var/2;

    cal_var |= 0x8000; // Set MSB

    ADC_MG_REG(adcmap) = ADC_MG_MG(cal_var);

    ADC_SC3_REG(adcmap) &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

    return(0);
}


void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
    ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
    ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
    ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1;
    ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
    ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
    ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
//ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;
    ADC_SC1_REG(adcmap,A)= ADC_CfgPtr->STATUS1A;
    ADC_SC1_REG(adcmap,B)= ADC_CfgPtr->STATUS1B;
}


void ADC_Read_Cal(ADC_MemMapPtr adcmap, tADC_Cal_Blk *blk)
{
    blk->OFS  = ADC_OFS_REG(adcmap);
    blk->PG   = ADC_PG_REG(adcmap);
    blk->MG   = ADC_MG_REG(adcmap);
    blk->CLPD = ADC_CLPD_REG(adcmap);
    blk->CLPS = ADC_CLPS_REG(adcmap);
    blk->CLP4 = ADC_CLP4_REG(adcmap);
    blk->CLP3 = ADC_CLP3_REG(adcmap);
    blk->CLP2 = ADC_CLP2_REG(adcmap);
    blk->CLP1 = ADC_CLP1_REG(adcmap);
    blk->CLP0 = ADC_CLP0_REG(adcmap);
    blk->CLMD = ADC_CLMD_REG(adcmap);
    blk->CLMS = ADC_CLMS_REG(adcmap);
    blk->CLM4 = ADC_CLM4_REG(adcmap);
    blk->CLM3 = ADC_CLM3_REG(adcmap);
    blk->CLM2 = ADC_CLM2_REG(adcmap);
    blk->CLM1 = ADC_CLM1_REG(adcmap);
    blk->CLM0 = ADC_CLM0_REG(adcmap);

}


void TFC_InitADC0()
{
    tADC_Config Master_Adc0_Config;


    SIM->SCGC6 |= (SIM_SCGC6_ADC0_MASK);

    //Lets calibrate the ADC. 1st setup how the channel will be used.


    Master_Adc0_Config.CONFIG1 = ADLPC_NORMAL           //No low power mode
                                 | ADC_CFG1_ADIV(ADIV_4) //divide input by 4
                                 | ADLSMP_LONG           //long sample time
                                 | ADC_CFG1_MODE(MODE_12)//single ended 8-bit conversion
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);

    Master_Adc0_Config.CONFIG2 = MUXSEL_ADCA // select the A side of the ADC channel.
                                 | ADACKEN_DISABLED
                                 | ADHSC_HISPEED
                                 | ADC_CFG2_ADLSTS(ADLSTS_2);//Extra long sample Time (20 extra clocks)


    Master_Adc0_Config.COMPARE1 = 00000; // Comparators don't matter for calibration
    Master_Adc0_Config.COMPARE1 = 0xFFFF;

    Master_Adc0_Config.STATUS2  = ADTRG_HW //hardware triggers for calibration
                                  | ACFE_DISABLED //disable comparator
                                  | ACFGT_GREATER
                                  | ACREN_ENABLED
                                  | DMAEN_DISABLED //Disable DMA
                                  | ADC_SC2_REFSEL(REFSEL_EXT); //External Reference

    Master_Adc0_Config.STATUS3 = CAL_OFF
                                 | ADCO_SINGLE
                                 | AVGE_ENABLED;
                              //   | ADC_SC3_AVGS(AVGS_4);

    Master_Adc0_Config.PGA =     0; // Disable the PGA


    // Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.

    Master_Adc0_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(31);


    ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc0_Config);  // config ADC

    // Calibrate the ADC in the configuration in which it will be used:
    ADC_Cal(ADC0_BASE_PTR);                    // do the calibration


    Master_Adc0_Config.STATUS2  = ACFE_DISABLED //disable comparator
                                  | ACFGT_GREATER
                                  | ACREN_ENABLED
                                  | DMAEN_DISABLED //Disable DMA
                                  | ADC_SC2_REFSEL(REFSEL_EXT); //External Reference

    Master_Adc0_Config.STATUS3 = CAL_OFF
                                 | ADCO_SINGLE;



    ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc0_Config);
}


void TFC_InitADC_System()
{

    TFC_InitADC0();


    //All Adc processing of the Pots and linescan will be done in the ADC0 IRQ!
    //A state machine will scan through the channels.
    //This is done to automate the linescan capture on Channel 0 to ensure that timing is very even
    CurrentADC_State =  ADC_STATE_INIT;

    //The pump will be primed with the TPM1 interrupt.  upon timeout/interrupt it will set the SI signal high
    //for the camera and then start the conversions for the pots.

   // NVIC_SetVector(ADC0_IRQn,(uint32_t)ADC0_Handler);
    NVIC_EnableIRQ(ADC0_IRQn);

}

extern "C" void ADC0_IRQHandler()
{
    uint8_t Junk;

    switch(CurrentADC_State) {
        default:
            Junk =  ADC0->R[0];
            break;

        case ADC_STATE_CAPTURE_POT_0:

            PotADC_Value[0] = ADC0->R[0];
            ADC0->CFG2  &= ~ADC_CFG2_MUXSEL_MASK; //Select the A side of the mux
            ADC0->SC1[0]  =  TFC_POT_1_ADC_CHANNEL | ADC_SC1_AIEN_MASK;
            CurrentADC_State = ADC_STATE_CAPTURE_POT_1;

            break;

        case ADC_STATE_CAPTURE_POT_1:

            PotADC_Value[1] = ADC0->R[0];
            ADC0->CFG2  |= ADC_CFG2_MUXSEL_MASK; //Select the B side of the mux
            ADC0->SC1[0]  =  TFC_BAT_SENSE_CHANNEL| ADC_SC1_AIEN_MASK;
            CurrentADC_State = ADC_STATE_CAPTURE_BATTERY_LEVEL;

            break;

        case ADC_STATE_CAPTURE_BATTERY_LEVEL:

            BatSenseADC_Value = ADC0->R[0];

            //Now we will start the sequence for the Linescan camera

            TAOS_CLK_HIGH;

            for(Junk = 0; Junk<TAOS_CLK_COUNT/2; Junk++) {
            }

            TAOS_SI_LOW;


            CurrentLineScanPixel = 0;
            CurrentLineScanChannel = 0;
            CurrentADC_State = ADC_STATE_CAPTURE_LINE_SCAN;
            ADC0->CFG2  |= ADC_CFG2_MUXSEL_MASK; //Select the B side of the mux
            ADC0->SC1[0] =  TFC_LINESCAN0_ADC_CHANNEL | ADC_SC1_AIEN_MASK;

            break;

        case ADC_STATE_CAPTURE_LINE_SCAN:

            if(CurrentLineScanPixel<128) {
                if(CurrentLineScanChannel == 0) {
                    LineScanImage0WorkingBuffer[CurrentLineScanPixel] = ADC0->R[0];
                    ADC0->SC1[0]  =  TFC_LINESCAN1_ADC_CHANNEL | ADC_SC1_AIEN_MASK;
                    CurrentLineScanChannel = 1;

                } else {
                    LineScanImage1WorkingBuffer[CurrentLineScanPixel] = ADC0->R[0];
                    ADC0->SC1[0]  =  TFC_LINESCAN0_ADC_CHANNEL | ADC_SC1_AIEN_MASK;
                    CurrentLineScanChannel = 0;
                    CurrentLineScanPixel++;

                    TAOS_CLK_LOW;
                    for(Junk = 0; Junk<TAOS_CLK_COUNT/2; Junk++) {
                    }
                    TAOS_CLK_HIGH;

                }

            } else {
                // done with the capture sequence.  we can wait for the PIT0 IRQ to restart

                TAOS_CLK_HIGH;

                for(Junk = 0; Junk<TAOS_CLK_COUNT/2; Junk++) {
                }

                TAOS_CLK_LOW;
                CurrentADC_State = ADC_STATE_INIT;

                //swap the buffer

                if(LineScanWorkingBuffer == 0) {
                    LineScanWorkingBuffer = 1;

                    LineScanImage0WorkingBuffer = &LineScanImage0Buffer[1][0];
                    LineScanImage1WorkingBuffer = &LineScanImage1Buffer[1][0];

                    TFC_LineScanImage0 = &LineScanImage0Buffer[0][0];
                    TFC_LineScanImage1 = &LineScanImage1Buffer[0][0];
                } else {
                    LineScanWorkingBuffer = 0;
                    LineScanImage0WorkingBuffer = &LineScanImage0Buffer[0][0];
                    LineScanImage1WorkingBuffer = &LineScanImage1Buffer[0][0];

                    TFC_LineScanImage0  = &LineScanImage0Buffer[1][0];
                    TFC_LineScanImage1  = &LineScanImage1Buffer[1][0];
                }

                TFC_LineScanImageReady++;
            }

            break;
    }

}

void TFC_InitLineScanCamera()
{
    SIM->SCGC5 |=   SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK; //Make sure the clock is enabled for PORTE;
    PORTE->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   //Enable GPIO on on the pin for the CLOCK Signal
    PORTD->PCR[7] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   //Enable GPIO on on the pin for SI signal

    PORTD->PCR[5] = PORT_PCR_MUX(0); //Make sure AO signal goes to an analog input
    PORTD->PCR[6] = PORT_PCR_MUX(0); //Make sure AO signal goes to an analog input

    //Make sure the Clock and SI pins are outputs
    PTD->PDDR |= (1<<7);
    PTE->PDDR |= (1<<1);

    TAOS_CLK_LOW;
    TAOS_SI_LOW;

    LineScanWorkingBuffer = 0;

    LineScanImage0WorkingBuffer = &LineScanImage0Buffer[LineScanWorkingBuffer][0];
    LineScanImage1WorkingBuffer = &LineScanImage1Buffer[LineScanWorkingBuffer][0];

    TFC_LineScanImage0 = &LineScanImage0Buffer[1][0];
    TFC_LineScanImage1  = &LineScanImage1Buffer[1][0];
}





/** Initialized TPM0 to be used for generating PWM signals for the the dual drive motors.   This method is called in the TFC constructor with a default value of 4000.0Hz
*
*  @param SwitchingFrequency PWM Switching Frequency in floating point format.   Pick something between 1000 and 9000.   Maybe you can modulate it and make a tune.
*/
void TFC_InitMotorPWM(float SwitchingFrequency)
{
    //Clock Setup for the TPM requires a couple steps.

    //1st,  set the clock mux
    //See Page 124 of f the KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual, Rev. 3, September 2012)
    SIM->SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual, Rev. 3, September 2012)


    //Enable the Clock to the FTM0 Module
    //See Page 207 of f the KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    //The TPM Module has Clock.  Now set up the peripheral

    //Blow away the control registers to ensure that the counter is not running
    TPM0->SC = 0;
    TPM0->CONF = 0;

    //While the counter is disabled we can setup the prescaler

    TPM0->SC = TPM_SC_PS(FTM0_CLK_PRESCALE);

    //Setup the mod register to get the correct PWM Period

    TPM0->MOD = (uint32_t)((float)(FTM0_CLOCK/(1<<FTM0_CLK_PRESCALE))/SwitchingFrequency);

    //Setup Channels 0,1,2,3
    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK; // invert the second PWM signal for a complimentary output;
    TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[3].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK; // invert the second PWM signal for a complimentary output;

    //Enable the Counter

    //Set the Default duty cycle to 50% duty cycle
    TFC_SetMotorPWM(0.0,0.0);

    //Enable the TPM COunter
    TPM0->SC |= TPM_SC_CMOD(1);

    //Enable the FTM functions on the the port
    PORTC->PCR[1] = PORT_PCR_MUX(4);
    PORTC->PCR[2] = PORT_PCR_MUX(4);
    PORTC->PCR[3] = PORT_PCR_MUX(4);
    PORTC->PCR[4] = PORT_PCR_MUX(4);

}

void TFC_SetMotorPWM(float MotorA , float MotorB)
{
    if(MotorA>1.0)
        MotorA = 1.0;
    else if(MotorA<-1.0)
        MotorA = -1.0;

    if(MotorB>1.0)
        MotorB = 1.0;
    else if(MotorB<-1.0)
        MotorB = -1.0;

    TPM0->CONTROLS[2].CnV = (uint16_t) ((float)TPM0->MOD * (float)((MotorA + 1.0)/2.0));
    TPM0->CONTROLS[3].CnV = TPM0->CONTROLS[2].CnV;
    TPM0->CONTROLS[0].CnV = (uint16_t) ((float)TPM0->MOD * (float)((MotorB + 1.0)/2.0));
    TPM0->CONTROLS[1].CnV = TPM0->CONTROLS[0].CnV;

}

//Pot Reading is Scaled to return a value of -1.0 to 1.0
float TFC_ReadPot(uint8_t Channel)
{
    if(Channel == 0)
        return ((float)PotADC_Value[0]/-((float)ADC_MAX_CODE/2.0))+1.0;
    else
        return ((float)PotADC_Value[1]/-((float)ADC_MAX_CODE/2.0))+1.0;
}

float TFC_ReadBatteryVoltage()
{
    return (((float)BatSenseADC_Value/(float)(ADC_MAX_CODE)) * 3.0);// * ((47000.0+10000.0)/10000.0);
}


void TFC_SetBatteryLED_Level(uint8_t BattLevel)
{
    switch(BattLevel)
    {
        default:
        case 0:
            TFC_BAT_LED0_OFF;
            TFC_BAT_LED1_OFF; 
            TFC_BAT_LED2_OFF; 
            TFC_BAT_LED3_OFF;
        break;
    
        case 1:
            TFC_BAT_LED0_ON;
            TFC_BAT_LED1_OFF; 
            TFC_BAT_LED2_OFF; 
            TFC_BAT_LED3_OFF;
        break;
        
        case 2:
            TFC_BAT_LED0_ON;
            TFC_BAT_LED1_ON; 
            TFC_BAT_LED2_OFF; 
            TFC_BAT_LED3_OFF;
        break;
        
        case 3:
            TFC_BAT_LED0_ON;
            TFC_BAT_LED1_ON; 
            TFC_BAT_LED2_ON; 
            TFC_BAT_LED3_OFF;
        break;
        
        case 4:
            TFC_BAT_LED0_ON;
            TFC_BAT_LED1_ON; 
            TFC_BAT_LED2_ON; 
            TFC_BAT_LED3_ON;
        break;
        
    }
}
