#include "mbed.h"

#include "Spices.h"
#include "common.h"
#include "TFC.h"

// camera params
#define NUM_LINE_SCAN 128
#define MAX_LINE_SCAN NUM_LINE_SCAN-1
#define MIN_LINE_WIDTH  0
#define MAX_LINE_WIDTH  15
#define FILTER_ENDS 0                              // # of pixels at end of camera data to ignore; set to 0 for now, later make 15
#define RANGE (NUM_LINE_SCAN - (2 * FILTER_ENDS))  // range of camera pixels to consider
#define ERR_RATIO 0.85                             // ratio of max possible error to pixels (have to measure!)
#define DER_RATIO 0.5                              // ratio for der threshold level (was 0.5 initially, may put back)

// steer/servo params
//#define MAX_STEER_LEFT -0.51                       // value determined by demo mode 1 measure (have to be adjusted with every servo horn attach)
//#define MAX_STEER_RIGHT 0.39                       // value determined by demo mode 1 measure
#define MAX_STEER_LEFT  -0.63
#define MAX_STEER_RIGHT  0.22
#define DT 0.02                                    // # MS of time between intervals (doesn't really matter)

// logging parameters
#define NUM_LOG_FRAMES 700                         // # of frames to log (when logging active) ~14 sec worth!

// ******  for debug tuning   ******
#define TUNE_SPEED 0.7
#define TUNE_KP 0.008
#define TUNE_KI 0
#define TUNE_KD 0
#define MIN_POWER 60                               // percent min power (estimating for a 2-ft turn => 24" / (24" + 6" car) = 4/5; speed of inner wheel is 20% lower worst case
#define SPEED_ADJUST 4
#define ABS_ERROR_THRESH 10                        // number of pixels line position offset before changing KP value
#define CONTROL_METHOD 2                           // which control method to use


// Drive/motor params
// 0.4 way too slow!!  need to charge battery
#define SUB_LIGHT_SPEED 0.5                        // moderate speed (value 0 to 1 sent to motors)
#define LIGHT_SPEED 0.6                            // fast...
#define RIDICULOUS_SPEED 0.7                       // faster...
#define LUDICROUS_SPEED 0.9                        // faster still!
#define MAX_POWER 100                              // percent max power (for speed adjustments)

// algo params
#define UNKNOWN_COUNT_MAX  50                      // max value to allow for unknown track conditions before killing engine
#define STARTGATEFOUNDMAX  0                       // max value to allow for finding starting gate before killing engine
#define STARTGATEDELAY     50                      // Delay before searching for starting gate to kill engine

#define MMA8451_I2C_ADDRESS (0x1d<<1)              // address for accelerometer?


/* CAR INTERFACE

  DIP SWITCH:
  -----------------------------------------------------------------
  1 - ON: Run MCP below; OFF: Run Demo program (see main.cpp)
  2 - ON: Log frame data to array
  3 - ON: Risky race option; OFF: Conservative race option
  4 - ON: Start Gate Kill Switch Active
  
  POTS
  -----------------------------------------------------------------
  0 - controls nothing at the moment
  1 - controls nothing at the moment

  PUSHBUTTONS  
  -----------------------------------------------------------------
  A -  START car race!
  B -  END CAR RACE / (while holding down when 'log frame data' active will also output terminal data)
  
*/

// LEARNING CAR CLUB 9/10/13: 
// IP need to test-- get around U turns -- more aggressive proportional control?  Or derivative?  Have camera look farther ahead but not too far ahead
// IP need to test -- fix lighting for tunnels (if good algo doesn't need lighting!!)
// -- increase power up to get up hills, brake down hills (accel: http://mbed.org/users/SomeRandomBloke/code/MMA8451Q/#)
// -- add speed control? (hall effect sensor)
// DONE -- make sure steering won't go past limits!!
// 9/12/13 - adjust camera exposure time based on maximum light intensity!!
// 9/14/13 - DONE -- make derivative threshold related to maximum light intensity!
// 9/16/13 - crash at car club blew out resistor R8, replaced it and getty twitching when powering servo from USB only.  Ok when powering from battery.
// 9/17/13 - experiments show that derivative control just doesn't work very well-- the tinest error delta causes huge drastic changes, need to use non-linear proportional
//           control instead... parabolic??
//           DONE -- Also need to slow down on curves
//           speed up hills and slow down on downhill...
//           measure speed?
// 9/18/13 - Test track work: doesn't appear to be sampling camera fast enough--  not able to handle the wiggly track and sometimes not able to handle the curves!
//           TODO: Need to increase rate at which camera sampled and decisions are made!!  Look to codewarrior code
//           Need to cut off left/right ends of camera data-- seems to not read line properly in well lit rooms
//           DONE Speed adjust as you go round; Need to have it slow down "into" curve, speed up again "out of curve"
// 9/23/13 - Definitely not processing camera fast enough-- seems to not react well when speed up the car.  Goes slow just fine all the way 'round.
//           TODO: Need to see how often TFC_LineScanImageReady gets updated. If update camera sample freq how will that impact exposure? Light adjustment algos should
//           be able to handle it. Need to be able to measure the 'processing time' required.  Also wondering if 20mS is fast enough for updating the servo?? Need
//           to calculate how fast servo needs to really go based on track curves and speed.
//           TODO: Need a way to measure speed I think as well.
//           TODO: Need to control speed differentially across the different motors!! See Eli video!!
//           TODO: Use this to get excel feedback quicker: http://strokescribe.com/en/serial-port-download.html  (Doesn't work too well-- prefer my own excel method)
//           TODO: Use PID control for steering!
//           TODO: Use Speed control (PID?) --- add speed sensor!
// 10/8/13   Reduced speed control to 90% (was 85%)-- seems to go off track and lose line at high speed-- mainly on the U turns
//           Need to figure out why derivative control in steering doesn't work well.  Add integral control.
//           Latest track times (no hill, no bumps, no tunnel, only squiggles) = 11.8sec at high speed with speed control
//           Losing time on the curves-- need to optimize!!
//           Worry about hill later-- need to get track times down around 8sec first.
//        *****************************
//        ** Implement some method to acount for U-TURNS on track-- and to help even go beyond what camera can see as far as estimating line position
//        ** (U-TURNS take the longest time out of track)
//        ** Method to help find the line even when not really visible
//        *****************************
// 10/17/13
//        NEED CURVE AND WIGGLE DETECT!!
// 10/20/13  Added logging capability and 'algo time' detect
//           It is not finding the line on the edges when on a curve-- likely because my line detect algo requires both edges
//           TODO-- need method that can use only one edge!

// TODO LIST
// - Speed Control via Sensor WAITING ON SENSOR
// - Differential drive around curves DONE -- still trying to figure out what %age to drop on turns
// - Full PID steering control IN PROGRESS
// - Starting gate kill engine debug IN PROGRESS -- need to add delay
// - Off track kill engine debug -- IN PROGRESS-- seems to kill car prematurely

// image processing vars
uint16_t   GrabLineScanImage0[NUM_LINE_SCAN];      // snapshot of camera data for this 'frame'
float      DerivLineScanImage0[NUM_LINE_SCAN];     // derivative of line scan data
float      NegEdges[NUM_LINE_SCAN];                // array-- set of where in line scan data negative edges found
float      PosEdges[NUM_LINE_SCAN];                // array-- set of where in line scan data positive edges found
uint16_t   numNegEdges = 0, numPosEdges = 0;       // max value of valid neg and positive indices (also serves as a count of # edges found)
uint16_t   MaxLightIntensity = 0;                  // max measured light intensity -- to account for lighting differences
uint16_t   MinLightIntensity = (1 << 12);          // min measured light intensity -- to account for lighting differences
float      maxDerVal = 0;                          // max deriv value
float      minDerVal = (float) (1 << 12);          // min deriv value
float      aveDerVal = 0;                          // average deriv value
float      DerivThreshold = (1 << 9);              // Derivative Threshold (default)
float      PosDerivThreshold = (1 << 9);           // Pos Edge Derivative Threshold (default)
float      NegDerivThreshold = (1 << 9);           // Neg Edge Derivative Threshold (default)


// Steering control variables
float      CurrentLinePosition;                    // Current position of track line (in pixels -- 0 to 127)
float      LastLinePosition;                       // Last position of track line (in pixels -- 0 to 127)
float      CurrentLinePosError = 0;                // Current line position error (used for derivative calc)
float      AbsError;
float      LastLinePosError = 0;                   // Last line position error (used for derivative calc)
float      SumLinePosError = 0;                    // Sum of line position error (used for integral calc)
float      DerivError = 0;                         // Derivative of error
float      CurrentSteerSetting = (MAX_STEER_RIGHT + MAX_STEER_LEFT) / 2;  // drive straight at first
float      CurrentLeftDriveSetting = 0;            // Drive setting (left wheel)
float      CurrentRightDriveSetting = 0;           // Drive setting (right wheel)

// Speed control vars
float      MaxSpeed;                                    // maximum speed allowed

uint16_t   startRaceTicker;                        // ticker at start of race1

// Custom Data Types
typedef enum TrackStatusType {Unknown,
                              LineFound,
                              StartGateFound,
                              LineJustLeft} TrackStatusType;
                              
TrackStatusType CurrentTrackStatus;                // current track status
TrackStatusType LastTrackStatus;                   // last track status                       

/* typedef enum TrackType {NotSure,
                        Straight,
                        Curve,
                        Wiggle,
                        Bumps,
                        StartGate,
                        UpHill,
                        DownHill} TrackType;

TrackType CurrentTrack; */


struct LogData {
  float linepos;
  float steersetting;
  float leftdrivesetting;
  float rightdrivesetting;
};

LogData    frameLogs[NUM_LOG_FRAMES];              // array of log data to store  
int        logDataIndex;                           // index for log data

int        StartGateFoundCount = 0;                // how many times start gate has been found
int        UnknownCount = 0;                       // how many times nothing has been found (to help with kill switch implementation)
bool       go = false;                             // Car can go!  Should be set to false to start.

// EXTRA CONTROL PARAMETERS
bool debugFakeMode = false;         // if true, ignores real camera and uses fake camera input instead; used for data processing debug
int terminalOutput = 2;             // set debug level for terminal output
                                    //    0 : no terminal output, race!
                                    //    1 : output just to measure frame rate
                                    //    2 : output for measuring time of operations
                                    //    3 : output with delay
bool doLogData = false;             // whether to capture log data to output later on
bool killSwitch = false;             // whether to enable Kill Switch (allow engine to stop after not finding track)
bool startGateStop = false;         // whether to stop or not depending on starting gate reading
bool doRisky = false;               // race style-- whether conservative or risky

// timer stuff
Timer timer;
int after_time, before_time, start_time, last_start_time;
bool run_once = false;

void ExecuteEngine() {

  // put here all things want to run only once after reset
  if (!run_once){
    if ((terminalOutput == 1) || (terminalOutput == 2)){
      timer.start();
    }
    run_once = true;
  }
  
  // read DIP switches and Pots for data
  readSwitches();
                                                          
  // Every 4s (or Terminal Output is off, i.e. race mode!)
  //    AND line scan image ready (or fake mode where image is always ready)
  //   (ticker updates every 2ms) (Line scan image ready very 20mS?)
  if((TFC_Ticker[0] > 2000 || (terminalOutput != 3)) && (TFC_LineScanImageReady > 0 || debugFakeMode))
   {

     // stuff that needs to be reset with each image frame
     if (terminalOutput == 1) {
       last_start_time = start_time;
       start_time = timer.read_us();
       TERMINAL_PRINTF("TIME:Between frames:%d:uSec\r\n", start_time - last_start_time);
       before_time = timer.read_us();
     }
     TFC_Ticker[0] = 0;
     TFC_LineScanImageReady=0; // must reset to 0 after detecting non-zero
     MaxLightIntensity = 0;                  // reset
     MinLightIntensity = (1 << 12);          // reset

     // grab camera frame
     grabCameraFrame();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER grabCameraFrame:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }
  
     // calcalate derivative of linescandata, filter starttime data
     derivativeLineScan(&GrabLineScanImage0[0], &DerivLineScanImage0[0]);

     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER derivativeLineScan:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }
            
     // adjust deriv threshold based on max lighting value
     // has to be called before find edges
     adjustLights();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER adjustLights:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     } 
         
     //find edges from derivative data
     findEdges_v2(&DerivLineScanImage0[0]);

     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER findEdges_v2:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     } 
     
     // turn on terminal output if line not found -- FOR DEBUG
     //if (CurrentTrackStatus == Unknown)
     //  terminalOutput = 1;

     //review edge data and set position or starting flag appropriately
     reviewEdges();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER reviewEdges:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }      
    
     if (terminalOutput == 3) {
       // print data to Terminal for camera 0
       printLineScanData(&GrabLineScanImage0[0]);

     // print deriviative of linescandata, filter starttime data
       printDerivLineScanData(&DerivLineScanImage0[0]);
       
       printAdjustLightsData();
       
       printEdgesFound();

     }
     
     // ** Track Status available at this point **  
      
     
     // test out accelerometer
    // accelTest();
      
     // Update things based on latest track status
     // e.g. change steering setting, stop car, ...
     ActOnTrackStatus();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER ActOnTrackStatus:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }      


     //give LED feedback as to track status
     feedbackLights();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER feedbackLights:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();       
     }  
     
     // control max power (speed)
     SpeedControl();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER SpeedControl:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }       

     // Drive!!     
     Drive();
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME:TO AFTER Drive:%d:uSec\r\n", after_time - before_time);
       before_time = timer.read_us();
     }         
     
     // wait_ms(1);

     // Capture Log data while driving
     if (go && doLogData) {
       captureData();
     }
     
     // Dump Log data to Terminal while stopped and holding B button
     if (!go && doLogData && TFC_PUSH_BUTTON_1_PRESSED) {
       dumpData();
     }
     
     if (terminalOutput == 2) {
       after_time = timer.read_us();
       TERMINAL_PRINTF("TIME: ENTIRE FRAME (include prints):%d:uSec\r\n", after_time - start_time);
       before_time = timer.read_us();
     }

     if (terminalOutput == 3) {
       TERMINAL_PRINTF("\r\n**************************END********************************\r\n");
     }

   } 
}

void dumpData() {
   TERMINAL_PRINTF("INDEX, LINEPOS, STEERSETTING, LEFTDRIVESETTING, RIGHTDRIVESETTING\r\n");
       for(logDataIndex = 0; logDataIndex < NUM_LOG_FRAMES; logDataIndex++) {
         TERMINAL_PRINTF("%d,%6.2f,%6.2f,%6.2f,%6.2f\r\n",logDataIndex,frameLogs[logDataIndex].linepos,frameLogs[logDataIndex].steersetting,frameLogs[logDataIndex].leftdrivesetting,frameLogs[logDataIndex].rightdrivesetting);
       }
}

void captureData()
{
  frameLogs[logDataIndex].linepos = CurrentLinePosition;
  frameLogs[logDataIndex].steersetting = CurrentSteerSetting;
  frameLogs[logDataIndex].leftdrivesetting = CurrentLeftDriveSetting;
  frameLogs[logDataIndex].rightdrivesetting = CurrentRightDriveSetting;

  // increment index                
  logDataIndex++;
  if (logDataIndex > NUM_LOG_FRAMES) 
    logDataIndex = 0;
}

void readSwitches()
{

  // ********* GATHER DIP SWITCH INPUTS *********
  if(TFC_GetDIP_Switch()&0x02)     // SWITCH 2 
    doLogData = true;              // Log data to array
  else
    doLogData = false;             // normal operation
  
  if(TFC_GetDIP_Switch()&0x04)     // SWITCH 3
    doRisky = true;     
  else
    doRisky = false;          
    
  if(TFC_GetDIP_Switch()&0x08)     // SWITCH 4 control start stop gate
    startGateStop = true;
  else
    startGateStop = false;


}

void grabCameraFrame()
{
  uint32_t i = 0;
  uint8_t fake_type = 4;                   // type of fake data if used

  for(i=0;i<NUM_LINE_SCAN;i++) // print one line worth of data (128) from Camera 0
  { 
  
    if (debugFakeMode) {                   // use fake camera data
      switch (fake_type) {
      case 0:                              // ideal track -- line in center
         if (i<57 || i > 70)
           GrabLineScanImage0[i] = 0xFFF;  // no line
         else
           GrabLineScanImage0[i] = 0x4B0;  // line
         break;
      case 1:                              // ideal track -- line to the left
         if (i<27 || i > 40)
           GrabLineScanImage0[i] = 0xFFF;  // no line
         else
           GrabLineScanImage0[i] = 0x4B0;  // line
         break;
      case 2:                              // ideal track -- line to the right
         if (i<87 || i > 100)
           GrabLineScanImage0[i] = 0xFFF;  // no line
         else
           GrabLineScanImage0[i] = 0x4B0;  // line
         break; 
      case 3:                              // ideal track -- starting gate!
         // TBD
         break;              
      case 4:                              // less than ideal track -- debug multi-edge issue!
         if (i<54)
           GrabLineScanImage0[i] = 4000;   // no line
         if (i == 54)
           GrabLineScanImage0[i] = 3370;   // neg edge
         if (i == 55)
           GrabLineScanImage0[i] = 3309;   // neg edge
         if (i == 56)
           GrabLineScanImage0[i] = 2016;   // neg edge
         if (i == 57)
           GrabLineScanImage0[i] = 711;    // neg edge
         if (i == 58)
           GrabLineScanImage0[i] = 696;    // neg edge
         if ((i>58) && (i<69))
           GrabLineScanImage0[i] = 500;    // line
         if (i == 69)
           GrabLineScanImage0[i] = 1800;   // pos edge
         if (i > 69)
           GrabLineScanImage0[i] = 4000;   // no line
      default:
         break;
      }
    
    } else {                               // use real camera data
      GrabLineScanImage0[i] = TFC_LineScanImage0[i];
    }
  }


}

void printLineScanData(uint16_t* LineScanData)
{
  uint32_t i = 0;
  float Val;

  TERMINAL_PRINTF("LINE SCAN DATA:,");

  for(i=0;i<NUM_LINE_SCAN;i++) // print one line worth of data (128) from Camera 0
  { 
    if (1 == 1) { // use float to print
      Val = (float) LineScanData[i];
      TERMINAL_PRINTF("%9.3f",Val);
      if(i==MAX_LINE_SCAN)  // when last data reached put in line return
        TERMINAL_PRINTF("\r\n");
      else
        TERMINAL_PRINTF(",");
    } else {         
      TERMINAL_PRINTF("0x%X",LineScanData[i]);
      if(i==MAX_LINE_SCAN)  // when last data reached put in line return
        TERMINAL_PRINTF("\r\n",LineScanData[i]);
      else
        TERMINAL_PRINTF(",",LineScanData[i]);
      }
  }

}

void printDerivLineScanData(float* derivLineScanData)
{
  uint32_t i, minCnt = 0, maxCnt = 0;
  
  minCnt = FILTER_ENDS;
  maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  TERMINAL_PRINTF("DERIVATIVE DATA:,");

  for(i=minCnt;i<maxCnt;i++) // print one line worth of data (128) from Camera 0
  {          
    TERMINAL_PRINTF("%9.3f",derivLineScanData[i]);
    if(i==maxCnt-1)          // when last data reached put in line return
      TERMINAL_PRINTF("\r\n",derivLineScanData[i]);
    else
      TERMINAL_PRINTF(", ",derivLineScanData[i]);
  }

}

void derivativeLineScan(uint16_t* LineScanDataIn, float* DerivLineScanDataOut)
{

  uint32_t i, minCnt = 0, maxCnt = 0;
  float DerVal, upperDerVal, lowerDerVal = 0;
  
  maxDerVal = 0;
  minDerVal = (float) (1 << 12);
  aveDerVal = 0;
  
  minCnt = FILTER_ENDS;
  maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  // TERMINAL_PRINTF("i, upperDerVal, lowerDerVal, DerVal\r\n");
  
  for(i=minCnt;i<maxCnt;i++) // print one line worth of data from Camera 0
  {  

     // store max light intensity value
     if (LineScanDataIn[i] > MaxLightIntensity)
       MaxLightIntensity = LineScanDataIn[i];

     // store min light intensity value
     if (LineScanDataIn[i] < MinLightIntensity)
       MinLightIntensity = LineScanDataIn[i];
       

     // Central Derivative
     if (i==minCnt) {                       // start point
       upperDerVal = (float)(LineScanDataIn[i+1]);
       lowerDerVal = (float)(LineScanDataIn[i]);  // make same as start point
     } else if (i==maxCnt - 1){             // end point
       upperDerVal = (float)(LineScanDataIn[i]);   // make same as end point
       lowerDerVal = (float)(LineScanDataIn[i-1]);
     } else {                               // any other point
       upperDerVal = (float)(LineScanDataIn[i+1]);
       lowerDerVal = (float)(LineScanDataIn[i-1]);
     }
     DerVal = (upperDerVal - lowerDerVal) / 2;
   //  TERMINAL_PRINTF("%d,%9.3f,%9.3f,%9.3f\r\n", i, upperDerVal, lowerDerVal, DerVal);
            
     if (DerVal > maxDerVal) {
       maxDerVal = DerVal;
     } 
     if (DerVal < minDerVal) {
       minDerVal = DerVal;
     }
     aveDerVal = aveDerVal + DerVal;           //get sum
     DerivLineScanDataOut[i] = DerVal;    
  }
  aveDerVal = (float) aveDerVal / (maxCnt - minCnt);
}

//Not reliable for finding edges!
void findEdges(float* derivLineScanData)
{
  // search for edges in deriviative data using a threshold
  // need to store in a hash if that's possible...
  // combine edges that are a pixel apart
 
  int i, minCnt = 0, maxCnt = 0;
  int multiNegEdgeCnt = 1, multiNegEdgeSum = 0;
  int multiPosEdgeCnt = 1, multiPosEdgeSum = 0;
      
  minCnt = FILTER_ENDS;
  maxCnt = NUM_LINE_SCAN - FILTER_ENDS;
  
  numNegEdges = 0;
  numPosEdges = 0;
  for(i=minCnt;i<maxCnt;i++) // print one line worth of data from Camera 0
  {  
     if (derivLineScanData[i] <= NegDerivThreshold)     // NEGATIVE EDGE FOUND!
     {
       if (terminalOutput == 3) {
         TERMINAL_PRINTF("NEG EDGE FOUND AT INDEX %d WITH VALUE %9.3f\r\n", i, derivLineScanData[i]);
       }
       
       if ((numNegEdges > 0) && (NegEdges[numNegEdges - 1] + 1 == i )) // if no multi edges
       {  // edge actually across multiple pixels
         multiNegEdgeCnt++;
         multiNegEdgeSum = multiNegEdgeSum + i;
         if (terminalOutput == 3) {
           TERMINAL_PRINTF("MULTIEDGE FOUND! MultiNegEdgeCnt: %d; MultiNegEdgeSum: %d\r\n", multiNegEdgeCnt, multiNegEdgeSum);
         }
       } else {  // not a multi-pixel edge known at this time, store negative edge index value
         numNegEdges++;
         if (terminalOutput == 3) {
           TERMINAL_PRINTF("NEG EDGE STORED WITH INDEX %d.  NUM NEG EDGES = %d\r\n", i, numNegEdges);
         }
         NegEdges[numNegEdges - 1] = (float) i;
         multiNegEdgeSum = i;
       }
 
 
     } else if (derivLineScanData[i] > PosDerivThreshold) {    // POSITIVE EDGE FOUND!
     
       if (terminalOutput == 3) {
         TERMINAL_PRINTF("POS EDGE FOUND AT INDEX %d WITH VALUE %9.3f\r\n", i, derivLineScanData[i]);
       }
       
       if ((numPosEdges > 0) && (PosEdges[numPosEdges - 1] + 1 == i ))
       {  // edge actually across multiple pixels
         multiPosEdgeCnt++;
         multiPosEdgeSum = multiPosEdgeSum + i;
         if (terminalOutput == 3) {
           TERMINAL_PRINTF("MULTIEDGE FOUND! MultiPosEdgeCnt: %d; MultiPosEdgeSum: %d\r\n", multiPosEdgeCnt, multiPosEdgeSum);
         }
       } else {  // not a multi-pixel edge known at this time, store Posative edge index value
         if (terminalOutput == 3) {
           TERMINAL_PRINTF("POS EDGE STORED WITH INDEX %d.  NUM POS EDGES = %d\r\n", i, numPosEdges);
         }
         numPosEdges++;
         PosEdges[numPosEdges - 1] = (float) i;
         multiPosEdgeSum = i;
       }      
       
     }  else {  // NO EDGE FOUND
       // combine multi-edges if there are any
       if (multiNegEdgeCnt > 1)
       { 
          NegEdges[numNegEdges - 1] = (float) multiNegEdgeSum / multiNegEdgeCnt;
          multiNegEdgeCnt = 1; multiNegEdgeSum = 0;
       }
       if (multiPosEdgeCnt > 1)
       { 
          PosEdges[numPosEdges - 1] = (float) multiPosEdgeSum / multiPosEdgeCnt;
          multiPosEdgeCnt = 1; multiPosEdgeSum = 0;
       }       
     
     }
  }

}


void findEdges_v2(float* derivLineScanData)
{
  // search for edges in deriviative data using a threshold
  // need to store in a hash if that's possible...
  // combine edges that are a pixel apart
 
  int i;
  
  int NegEdgeBufCnt = 0, NegEdgeBufSum = 0;     // serves as buffer to store neg edges found next to each other
  int PosEdgeBufCnt = 0, PosEdgeBufSum = 0;     // serves as buffer to store pos edges found next to each other
      
  int minCnt = FILTER_ENDS;
  int maxCnt = NUM_LINE_SCAN - FILTER_ENDS;
  
  
  
  numNegEdges = 0;                              // count of neg edges found thus far
  numPosEdges = 0;                              // count of pos edges found thus far
  for(i=minCnt;i<maxCnt;i++) // print one line worth of data from Camera 0
  {  
     
     if (derivLineScanData[i] <= NegDerivThreshold)          // NEGATIVE EDGE FOUND!
     {
       
       if (terminalOutput == 3) {
         TERMINAL_PRINTF("NEG EDGE FOUND AT INDEX %d WITH VALUE %9.3f\r\n", i, derivLineScanData[i]);
       }

       NegEdgeBufCnt++;                                      // add value to neg edge buffer
       NegEdgeBufSum = NegEdgeBufSum + i;
       
     } else if (derivLineScanData[i] > PosDerivThreshold) {  // POSITIVE EDGE FOUND!
       
       if (terminalOutput == 3) {
         TERMINAL_PRINTF("POS EDGE FOUND AT INDEX %d WITH VALUE %9.3f\r\n", i, derivLineScanData[i]);
       }

       PosEdgeBufCnt++;                                      // add value to pos edge buffer
       PosEdgeBufSum = PosEdgeBufSum + i;
       
     }  else {                                               // NO EDGE FOUND
         
       // POP EDGE BUFFERS IF NON-EMPTY AND STORE TO EDGE "STACK" (i.e. edges found)
       
       if (NegEdgeBufCnt > 0) {
         // store edge value
         numNegEdges++;
         NegEdges[numNegEdges - 1] = (float) NegEdgeBufSum / NegEdgeBufCnt;
         
         // clear edge buffer      
         NegEdgeBufSum = 0; NegEdgeBufCnt = 0;
       }

       if (PosEdgeBufCnt > 0) {
         // store edge value
         numPosEdges++;
         PosEdges[numPosEdges - 1] = (float) PosEdgeBufSum / PosEdgeBufCnt;
         
         // clear edge buffer
         PosEdgeBufSum = 0; PosEdgeBufCnt = 0;
       }        
     
     }
     
  }

}

void printEdgesFound()
{
  int i;
  
    // Check that neg edges captured ok
    TERMINAL_PRINTF("NEGATIVE EDGES FOUND:,");
    for(i=0;i<=numNegEdges-1;i++)
    {
      TERMINAL_PRINTF("%9.3f",NegEdges[i]);
      if(i==numNegEdges-1)              // when last data reached put in line return
        TERMINAL_PRINTF("\r\n");
      else
        TERMINAL_PRINTF(", ");
    }
  
  
    // Check that pos edges captured ok
    TERMINAL_PRINTF("POSITIVE EDGES FOUND:,");
    for(i=0;i<=numPosEdges-1;i++)
    {
      TERMINAL_PRINTF("%9.3f",PosEdges[i]);
      if(i==numPosEdges-1)              // when last data reached put in line return
        TERMINAL_PRINTF("\r\n");
      else
        TERMINAL_PRINTF(", ");
    }

}

void reviewEdges()
{
  LastTrackStatus = CurrentTrackStatus;
  
  if ((numPosEdges == 1) && (numNegEdges == 1))  // only one negative and positive edge found (LINE)
  {
    if (((PosEdges[0] - NegEdges[0]) >= MIN_LINE_WIDTH) && ((PosEdges[0] - NegEdges[0]) <= MAX_LINE_WIDTH)) // has proper expected width
    {
       CurrentTrackStatus = LineFound;                                   // report line found!
       UnknownCount = 0;                                          // reset unknown status count
       LastLinePosition = CurrentLinePosition;
       CurrentLinePosition = (PosEdges[0]+NegEdges[0]) / 2;       // update line position
    }
  } else if ((numPosEdges == 1) && (numNegEdges == 0))  {     // 1 pos edge found (POSSIBLE LINE)
    if ((PosEdges[0] <= MAX_LINE_WIDTH) && (LastLinePosError < 0))       // pos edge is within line width of edge of camera (LEFT)
    {
       CurrentTrackStatus = LineFound;                                   // report line found!
       UnknownCount = 0;                                                 // reset unknown status count
       LastLinePosition = CurrentLinePosition;
       CurrentLinePosition = PosEdges[0] - ( MAX_LINE_WIDTH / 2);        // update line position
     //  TERMINAL_PRINTF("*** SINGLE POSEDGE LINE FOUND AT POSITION %9.3f *** \r\n", CurrentLinePosition);
    }
  } else if ((numNegEdges == 1) && (numPosEdges == 0))  {     // 1 neg edge found (POSSIBLE LINE)
    if ((NegEdges[0] >= (MAX_LINE_SCAN - MAX_LINE_WIDTH)) && (LastLinePosError > 0))    // neg edge is within line width of edge of camera (RIGHT)
    {
       CurrentTrackStatus = LineFound;                                   // report line found!
       UnknownCount = 0;                                                 // reset unknown status count
       LastLinePosition = CurrentLinePosition;
       CurrentLinePosition = NegEdges[0] + ( MAX_LINE_WIDTH / 2);        // update line position
    //   TERMINAL_PRINTF("*** SINGLE NEGEDGE LINE FOUND AT POSITION %9.3f *** \r\n", CurrentLinePosition);
    } 
  } else if ((numPosEdges == 2) && (numNegEdges == 2))  {     // 2 negative and 2 positive edges found (STARTING/FINISH GATE)
  
    if ((((NegEdges[0] - PosEdges[0]) >= MIN_LINE_WIDTH) && ((NegEdges[0] - PosEdges[0]) <= MAX_LINE_WIDTH)) &&    // white left 'line'
        (((NegEdges[1] - PosEdges[1]) >= MIN_LINE_WIDTH) && ((NegEdges[1] - PosEdges[1]) <= MAX_LINE_WIDTH)) &&    // white right 'line'
        (((PosEdges[1] - NegEdges[0]) >= MIN_LINE_WIDTH) && ((PosEdges[1] - NegEdges[0]) <= MAX_LINE_WIDTH))       // actual track line
        )
           
       
    if (startRaceTicker > STARTGATEDELAY) {                      // only start counting for starting gate until after delay
      StartGateFoundCount++;
    }
    
    CurrentTrackStatus = StartGateFound;
    UnknownCount = 0;                                            // reset unknown status count
           
  } else if ((numPosEdges > 1) && (numNegEdges > 1)) {   // more than 1 negative edge and positive edge found (but not 2 for both) (STARTING / FINISH GATE)
  
   // remove edges that aren't close to center TBD DDHH
   
      if (terminalOutput == 3) {
         TERMINAL_PRINTF("***************************************** \r\n");
         TERMINAL_PRINTF("********** NOT SURE FOUND ********** \r\n");
         TERMINAL_PRINTF("***************************************** \r\n");
       } 
    CurrentTrackStatus = Unknown; 
  
  } else {  // no track or starting gate found
  
    if (terminalOutput == 3) {
      TERMINAL_PRINTF("***************************************** \r\n");
      TERMINAL_PRINTF("*** !!!!!!!!!! LINE NOT FOUND !!!!!!! *** \r\n", CurrentLinePosition);
      TERMINAL_PRINTF("***************************************** \r\n");
    }
  
    CurrentTrackStatus = Unknown;
    UnknownCount++;
  }
  
  


}

void ActOnTrackStatus()
{
  // Decide what to do next based on current track status

  if (CurrentTrackStatus == LineFound)   {             // LINE FOUND!
  
    if (terminalOutput == 3) {
      TERMINAL_PRINTF("***************************************** \r\n");
      TERMINAL_PRINTF("*** LINE FOUND AT POSITION %9.3f *** \r\n", CurrentLinePosition);
      TERMINAL_PRINTF("***************************************** \r\n");
    }
  
    // Update steering position 
    SteeringControl();

    // Apply to servo    
    Steer();
    
  } else if (CurrentTrackStatus == StartGateFound) {   // STARTING GATE FOUND
  
    if (terminalOutput == 3) {
      TERMINAL_PRINTF("***************************************** \r\n");
      TERMINAL_PRINTF("********** STARTING GATE FOUND ********** \r\n");
      TERMINAL_PRINTF("**********     count = %d      ********** \r\n", StartGateFoundCount);
      TERMINAL_PRINTF("***************************************** \r\n");
    }
    
    // END RACE!
    if (startGateStop) {
      if (StartGateFoundCount > STARTGATEFOUNDMAX)
      {
       go = false;   // STOP!!
      } 
    }
  
  }
  


}

void SteeringControl()
{

  float targetPosition = (float)( (NUM_LINE_SCAN / 2) - 0.5);  // target to achieve for line position

  float KP;                                                    // proportional control factor
  float KI;                                                    // integral control factor
  float KD;                                                    // derivative control factor
  
  float Pout, Iout, Dout;                                      // PID terms
  
  // Calculate error
  // make error to the right positive
  // i.e. if LINE to the right-- then CurrentLinePosError > 0
  //      if LINE to the left -- then CurrentLinePosError < 0
  CurrentLinePosError = CurrentLinePosition - targetPosition;

  // Get absolute error
  if (CurrentLinePosError >= 0) 
    AbsError = CurrentLinePosError;
  else
    AbsError = -1 * CurrentLinePosError;

  // CHOOSE SET OF PID CONTROL PARAMETERS
  switch (CONTROL_METHOD) {
    case 0:
      // Pure proportional control based on range of steering values vs. range of error values
      KP = (float) ( MAX_STEER_RIGHT - MAX_STEER_LEFT ) / ( NUM_LINE_SCAN - (2*FILTER_ENDS) - MIN_LINE_WIDTH );
      KD = 0;
      KI = 0;
      break;
   case 1:
      // Proportional control with 50% bit more oomph --- a bit more aggressive around the bends
      KP = (float) ( MAX_STEER_RIGHT - MAX_STEER_LEFT ) / ( NUM_LINE_SCAN - (2*FILTER_ENDS) - MIN_LINE_WIDTH );
      KP = KP * 1.5;
      KD = 0;
      KI = 0;
      break;
    case 2:  // MANUAL TUNING CASE 1 (use pot to help determine tuning parameters)
      KP = TUNE_KP;
      KI = TUNE_KI;
      KD = TUNE_KD;
    case 3:
      if (AbsError < ABS_ERROR_THRESH) {
        KP = 0.003;  // when relatively straight, keep KP gain low
      } else {
        KP = 0.010;  // when curve begins or off track, increase KP gain
      }
      KI = 0;
      KD = 0;
      
    default:
      break;
  }
  
  /* Pseudocode
   previous_error = 0
   integral = 0 
   start:
     error = setpoint - measured_value
     integral = integral + error*dt
     derivative = (error - previous_error)/dt
     output = Kp*error + Ki*integral + Kd*derivative
     previous_error = error
     wait(dt)
     goto start 
  */
  
  
  if (terminalOutput == 3) {
    TERMINAL_PRINTF("KP = %6.4f\r\n", KP);
    TERMINAL_PRINTF("TARGET %6.3f\r\n", targetPosition);
  }
  


    // Update integral of error
    // i.e. if LINE stays to the right, then SumLinePosError increases
    // i.e. if LINE stays to the left, then SumLinePosError decreases
    SumLinePosError = SumLinePosError + ( CurrentLinePosError * DT );

    DerivError = (CurrentLinePosError - LastLinePosError) / DT;
    
    if (terminalOutput == 3) {
      TERMINAL_PRINTF("CURRENT LINE POSITION %9.3f\r\n", CurrentLinePosition);
      TERMINAL_PRINTF("CURRENT LINE POSITION ERROR %9.3f\r\n", CurrentLinePosError);
    }
    
    // SECOND- calculate new servo position
    
    // proportional control term
    Pout = KP * CurrentLinePosError;

    // integral control term
    Iout = KI * SumLinePosError;

    // Derivative control term
    Dout = KD * DerivError;

    if (terminalOutput == 3) {
      TERMINAL_PRINTF("KP = %6.4f\r\n", KP);
      TERMINAL_PRINTF("KI = %6.4f\r\n", KI);
      TERMINAL_PRINTF("KD = %6.4f\r\n", KD);
      TERMINAL_PRINTF("Pout = %6.4f\r\n", Pout);
      TERMINAL_PRINTF("Iout = %6.4f\r\n", Iout);
      TERMINAL_PRINTF("Dout = %6.4f\r\n", Dout);
    }

    // Finally add offset to steering to account for non-centered servo mounting
    // CurrentSteerSetting = Pout + Iout + Dout + ( (float) (MAX_STEER_LEFT + MAX_STEER_RIGHT) / 2 );
    CurrentSteerSetting = Pout + ( (float) (MAX_STEER_LEFT + MAX_STEER_RIGHT) / 2 );
    
    // store for next cycle deriv calculation
    LastLinePosError = CurrentLinePosError;

    // for tuning control algo only
    if (1 == 0) {
      TERMINAL_PRINTF("*** ******************************** \r\n");
      TERMINAL_PRINTF("*** LINE FOUND AT POSITION %9.3f *** \r\n", CurrentLinePosition);
      TERMINAL_PRINTF("*** ERROR %9.3f *** \r\n", CurrentLinePosError);
      TERMINAL_PRINTF("*** INTEGRAL ERROR %9.3f *** \r\n", SumLinePosError);
      TERMINAL_PRINTF("*** DERIVATIVE ERROR %9.3f *** \r\n", DerivError);
      TERMINAL_PRINTF("*** P STEER SETTING %9.3f *** \r\n", CurrentSteerSetting);
      TERMINAL_PRINTF("*** PI STEER SETTING  %9.3f *** \r\n", (CurrentSteerSetting + Iout));
      TERMINAL_PRINTF("*** ******************************** \r\n");
      wait_ms(1000);
    }

}

void Steer()
{

    // make sure doesn't go beyond steering limits
    if (CurrentSteerSetting > MAX_STEER_RIGHT)
    { 
       CurrentSteerSetting = MAX_STEER_RIGHT;
    } else if (CurrentSteerSetting < MAX_STEER_LEFT)
    {
       CurrentSteerSetting = MAX_STEER_LEFT;
    }

    if (terminalOutput == 3) {
      TERMINAL_PRINTF("APPLYING SERVO SETTING %5.3f\r\n", CurrentSteerSetting);
    }
    TFC_SetServo(0,CurrentSteerSetting);  

}

void SpeedControl()
{

  // Get max speed setting from reading pot0
  // then adjust
  
  float ErrLimit;
  float LeftDriveRatio, RightDriveRatio;
  
  // set maximum speed allowed
  switch (1)
    {
      case 0:
        // read value off pot0
        MaxSpeed = TFC_ReadPot(0);
        break;
      case 1:
        if (doRisky)
          MaxSpeed = TUNE_SPEED + 0.1;
        else 
          MaxSpeed = TUNE_SPEED;
        break;
      case 2:
        MaxSpeed = SUB_LIGHT_SPEED;
        break;
      case 3:
        MaxSpeed = LIGHT_SPEED;
        break;
      case 4:
        MaxSpeed = RIDICULOUS_SPEED;      
        break;
      case 5:
        MaxSpeed = LUDICROUS_SPEED;      
        break;        
      default:
        break;
    }
    
  switch (SPEED_ADJUST)
   {
     case 0:
       // SPEED ADJUST METHOD 0
       // no speed adjust
       LeftDriveRatio = MAX_POWER;
       RightDriveRatio = LeftDriveRatio;
     case 1:
       // SPEED ADJUST METHOD 1
       // High speed when error is low, low speed when error is high
       // lower speed when more than third outside of center
       ErrLimit = ((float) RANGE ) * 0.5 * ERR_RATIO * 0.33;
       if (AbsError > ErrLimit) {
         LeftDriveRatio = MIN_POWER;
       } else {
         LeftDriveRatio = MAX_POWER;
       }
       RightDriveRatio = LeftDriveRatio;
       break;
     case 2:
       // SPEED ADJUST METHOD 2
       // Have max/min speed adjust proportional to absolute value of line error
       ErrLimit = ((float) RANGE )  * 0.5 * ERR_RATIO; 
       LeftDriveRatio = MAX_POWER - ((MAX_POWER - MIN_POWER) * (AbsError / ErrLimit));
       RightDriveRatio = LeftDriveRatio;
       break;
     case 3:
       // SPEED ADJUST METHOD 3
       // have wheel relative speed proportional to absolute value of line error
       ErrLimit = ((float) RANGE )  * 0.5 * ERR_RATIO;
       if (CurrentLinePosError > 0) {           // heading right
         LeftDriveRatio = MAX_POWER;
         RightDriveRatio = (MIN_POWER - MAX_POWER) * (CurrentLinePosError * 2 / ( (float) RANGE ) ) + MAX_POWER;
       } else if (CurrentLinePosError < 0) {    // heading left
         RightDriveRatio = MAX_POWER;
         LeftDriveRatio = (MAX_POWER - MIN_POWER) * (CurrentLinePosError * 2 / ( (float) RANGE ) ) + MAX_POWER;
       } else {
         LeftDriveRatio = MAX_POWER;
         RightDriveRatio = MAX_POWER;
       }
       break;
     case 4:
       // SPEED ADJUST METHOD 4
       // have wheel relative speed proportional to absolute value of line error
       // only when above a certain error
       ErrLimit = ((float) RANGE )  * 0.5 * ERR_RATIO * 0.1;
       if (CurrentLinePosError > ErrLimit) {           // heading right
         LeftDriveRatio = MAX_POWER - (MAX_POWER - MIN_POWER) * (CurrentLinePosError * 2 / ( (float) RANGE ) );
         RightDriveRatio = MIN_POWER;
       } else if (CurrentLinePosError < (-1 * ErrLimit)) {    // heading left
         RightDriveRatio = MAX_POWER - (MAX_POWER - MIN_POWER) * (CurrentLinePosError * 2 / ( (float) RANGE ) );
         LeftDriveRatio = MIN_POWER;
       } else {
         LeftDriveRatio = MAX_POWER;
         RightDriveRatio = MAX_POWER;
       }
       break; 
     case 5:
       // SPEED ADJUST METHOD 5
       // High speed when error is low, low speed when error is high
       // lower speed when more than third outside of center
       ErrLimit = ((float) RANGE ) * 0.5 * ERR_RATIO * 0.2;
       if (AbsError > ErrLimit) {
         LeftDriveRatio = MIN_POWER;
       } else {
         LeftDriveRatio = MAX_POWER;
       }
       RightDriveRatio = LeftDriveRatio;
       break;   
     case 6:
       // SPEED ADJUST METHOD 6
       // High speed when error is low, low speed when error is high
       // lower speed when more than third outside of center
       if (AbsError > ABS_ERROR_THRESH) {
         LeftDriveRatio = MIN_POWER;
       } else {
         LeftDriveRatio = MAX_POWER;
       }
       RightDriveRatio = LeftDriveRatio;
       break;                 
     default:
       break;
       
  }
  // TBD-- add speed adjust based on Xaccel sensor!


  // currently no control mechanism as don't have speed sensor  
  CurrentLeftDriveSetting = (float) (LeftDriveRatio / 100) * MaxSpeed * -1;
  CurrentRightDriveSetting = (float) (RightDriveRatio / 100) * MaxSpeed * -1;

  
  if (terminalOutput == 3) {
    TERMINAL_PRINTF("Abs Error: %4.2f\r\n", AbsError);
    TERMINAL_PRINTF("Error Limit: %4.2f\r\n", ErrLimit);
    TERMINAL_PRINTF("MAX SPEED = %5.2f\n", MaxSpeed);
    TERMINAL_PRINTF("Current Left Drive Setting: %5.2f\r\n", CurrentLeftDriveSetting);
    TERMINAL_PRINTF("Current Right Drive Setting: %5.2f\r\n", CurrentRightDriveSetting);
  }
  if (1 == 0) {
   TERMINAL_PRINTF("Current Left Drive Setting: %5.2f\r\n", CurrentLeftDriveSetting);
   TERMINAL_PRINTF("Current Right Drive Setting: %5.2f\r\n", CurrentRightDriveSetting);
  }

}

void Drive()
{

  // START!
  // if not going, go when button A is pressed
  if (!go) {
    if(TFC_PUSH_BUTTON_0_PRESSED) {
      go = true;
      UnknownCount = 0;
      StartGateFoundCount = 0;
      startRaceTicker = TFC_Ticker[0];  // keep track of start of race
      logDataIndex = 0;                 // reset log data index
    }
  }
  
  // STOP!
  // if going, stop when button A is pressed
  if (go) {              
   if(TFC_PUSH_BUTTON_1_PRESSED) {
      go = false;
      StartGateFoundCount = 0;
    }
  }

  // EMERGENCY STOP!
  // 'kill switch' to prevent crashes off-track
  if (killSwitch) {
    if (UnknownCount > UNKNOWN_COUNT_MAX) {  // if track not found after certain time
      go = false;                            // kill engine
      StartGateFoundCount = 0;
    }
  }

// ****************
  
  if (!go) { // stop!
    TFC_SetMotorPWM(0,0); //Make sure motors are off 
    TFC_HBRIDGE_DISABLE;
  }

  if (go) {  // go!
    TFC_HBRIDGE_ENABLE;
    // motor A = right, motor B = left based on way it is mounted
    TFC_SetMotorPWM(CurrentRightDriveSetting,CurrentLeftDriveSetting);
  }
}


void adjustLights() {
    // LIGHT ADJUST METHOD 1
    // threshold is 1/5 of light intensity 'range'
    if (1 == 0) {
        DerivThreshold = (float) (MaxLightIntensity - MinLightIntensity) / 5;
        NegDerivThreshold = (float) -1 * (DerivThreshold);
        PosDerivThreshold = (float) (DerivThreshold);
    } 
    else {
        // LIGHT ADJUST METHOD 2 -- SEEMS TO WORK MUCH BETTER
        // pos edge threshold is half range of max deriv above aver derive
        // neg edge threshold is half range of min deriv above aver derive

        NegDerivThreshold = (float) (minDerVal - aveDerVal) * DER_RATIO;
        PosDerivThreshold = (float) (maxDerVal - aveDerVal) * DER_RATIO;

    }

    printAdjustLightsData();
}

void printAdjustLightsData() {
    if (terminalOutput == 3) {
        TERMINAL_PRINTF("Max Light Intensity: %4d\r\n", MaxLightIntensity);
        TERMINAL_PRINTF("Min Light Intensity: %4d\r\n", MinLightIntensity);
        TERMINAL_PRINTF("Deriv Threshold: %9.3f\r\n", DerivThreshold);
    }
}

void feedbackLights() {
    switch (CurrentTrackStatus) {
        case LineFound:
            TFC_BAT_LED0_OFF;
            TFC_BAT_LED1_ON;     
            TFC_BAT_LED2_ON;
            TFC_BAT_LED3_OFF;
        break;

        case StartGateFound:
            TFC_BAT_LED0_ON;
            TFC_BAT_LED1_OFF;     
            TFC_BAT_LED2_OFF;
            TFC_BAT_LED3_ON;   
        break;

        default:
            TFC_BAT_LED0_OFF;
            TFC_BAT_LED1_OFF;     
            TFC_BAT_LED2_OFF;
            TFC_BAT_LED3_OFF;
    }
    
}

