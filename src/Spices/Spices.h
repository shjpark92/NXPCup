#include "mbed.h"

#ifndef _SPICES_H
#define _SPICES_H

// *******************************************
// MASTER CONTROL PROGRAM

// Loop at Servo cycle (currently 50Hz / 20mS)
// Camera gets updated at its own period
//   (currently 50Hz / 20mS)
// 
//   REQUIREMENTS:
//   - Control servos in response to track line position
//   - Gather as much camera data as possible between
//     servo update cycles to get best read on the line
//     (currently 1:1 though @ 50Hz)
//   - Ignore erronous signals:
//       - "mud" on track
//       - low contrast light situations
//       - track crossing itself (all black situation)
//   - Detects 'starting line' -- ignores first detect (START)
//     and stops after second detect (FINISH)
//
//   INPUTS:
//   - linescancamera data (0-127)
//   - maximum speed
//   - filter settings:
//       A = 15 (number of pixels at each end to ignore)
//       B = 20 (estimated width of line)(possible issue if line far away)
//       C = 2^12 (max value based on ADC sample rate of 12 bits)
//       D = Edge threshold of Derivative (max / 4 = 2^10)
//   - control parameters, KP and KD
//
//   CONTROL ALGORITHM:
//   1 - Get Line Position
//   2 - Compute Error from Line Position
//   3 - Set Servo position based on Error and Derivative of Error
//   4 - Store error for next cycle to perform derivative
//   
//
//   GET LINE POSITION
//     INPUTS: linescan data
//     RETURNS: either position or starting gate flag
//   ALGO:
//   - Find line edges:
//      - filter out first and last A number of pixels from left and right
//      - calculate derivative of data
//      - search for line edges in derivative data (search for either track line or starting gate)
//        - Search for all pixels with Value < -D, indicating negative edge (large negative means going from white to black)
//        - Search for all pixels with Value > D, indicating positive edge (large positive means going from black to white)
//        - Clean up adjacent line edges-- edges 1 pixel away from each other combined to be considered a single edge, average pixel value
//   - if starting gate then send up flag
//   - if track, calculate position
//
//
//   COMPUTE LINE ERROR
//   - take line position and target-- calculate error (negative or positive)
//
//   SET SERVO POSITION 
//      -  int servoPosition = KP * Error + KD * (Error - lastError);
//      -  lastError = Error;
//
//
//   TBD:
//   - store multiple edge positions in order to average prior to next servo update
//
// *******************************************
//

// Track Params
#define CENTERLINE 0
#define EDGELINE 1
#define TRACKTYPE 1                                // 0 = center line track; 1 = edge line track

// Steer/Servo Params
#define MAX_STEER_LEFT -0.42                       // value determined by demo mode 1 measure (have to be adjusted with every servo horn attach)
#define MAX_STEER_RIGHT 0.43                       // value determined by demo mode 1 measure
#define DT 0.02                                    // # MS of time between intervals (doesn't really matter)

// Logging parameters
#define NUM_LOG_FRAMES 700                         // # of frames to log (when logging active) ~14 sec worth!

// ******  for debug tuning   ******
#define TUNE_KP 0.008
#define TUNE_KI 0
#define TUNE_KD 0
#define MIN_POWER 60                               // percent min power (estimating for a 2-ft turn => 24" / (24" + 6" car) = 4/5; speed of inner wheel is 20% lower worst case
#define SPEED_ADJUST 4                             // do not change
#define ABS_ERROR_THRESH 10                        // number of pixels line position offset before changing KP value
#define CONTROL_METHOD 2                           // which control method to use

// Drive/motor params
#define LIGHT_SPEED 0.4                            // easy speed
#define LUDICROUS_SPEED 0.6                        // faster speed
#define MAX_POWER 100                              // percent max power (for speed adjustments)

// Algo params
#define UNKNOWN_COUNT_MAX  50                      // max value to allow for unknown track conditions before killing engine
#define STARTGATEFOUNDMAX  0                       // max value to allow for finding starting gate before killing engine
#define STARTGATEDELAY     50                      // Delay before searching for starting gate to kill engine

// Camera Params
#define NUM_LINE_SCAN 128
#define MAX_LINE_SCAN NUM_LINE_SCAN-1
#define MIN_LINE_WIDTH  0
#define MAX_LINE_WIDTH  15
#define FILTER_ENDS 0                              // # of pixels at end of camera data to ignore; set to 0 for now, later make 15
#define RANGE (NUM_LINE_SCAN - (2 * FILTER_ENDS))  // range of camera pixels to consider
#define ERR_RATIO 0.85                             // ratio of max possible error to pixels (have to measure!)
#define DER_RATIO 0.75                             // ratio for der threshold level (was 0.5 initially, may put back)
#define LEFTLINEOFFSET 60                          // amount track center offset from left line if found (edge line track)
#define RIGHTLINEOFFSET 60                         // amount track center offset from right line if found (edge line track)

// ****************
// ** DATA TYPES **
// ****************

struct LogData {
  float linepos;
  float steersetting;
  float leftdrivesetting;
  float rightdrivesetting;
};

typedef enum TrackStatusType {Unknown,
                              LineFound,
                              StartGateFound,
                              LineJustLeft} TrackStatusType;
                          
/* typedef enum TrackType {NotSure,
                        Straight,
                        Curve,
                        Wiggle,
                        Bumps,
                        StartGate,
                        UpHill,
                        DownHill} TrackType;

TrackType CurrentTrack; */


// *************************
// ** FUNCTION PROTOTYPES **
// *************************

// review edge data
//   - report line position if there
//   - set starting gate flag if there
//   - do nothing with position value otherwise
void reviewEdgesCenterLine();
void reviewEdgesEdgeLine();

void resetImageProcessVars();

// prints out line scan data for you
void printLineScanData(uint16_t* LineScanData, const char* message);

// calculates derivative of line scan data
void derivativeLineScan(uint16_t* LineScanDataIn, float* DerivLineScanDataOut);

// prints out derivative of line scan data -- assumes deriv already calculated
void printDerivLineScanData(float* derivLineScanData, const char* message);

// find negative and positive edges in image data, store in array, combines edges found in adjacent pixels
void findEdges(float* derivLineScanData);

// improved algo to find negative and positive edges in image data, store in array, combines edges found in adjacent pixels
void findEdges_v2(float* derivLineScanData);

// prints out edge data found
void printEdgesFound();

void TrackMode();

uint16_t getMode();

bool terminalMode();


// serves to grab a frame of camera data
void grabCameraFrame();


// Decide new actions based on track status
void ActOnTrackStatus();

// Update Steering settings
void SteeringControl();

// Apply steering setting to servo!
void Steer();

// Update speed settings
void SpeedControl();

// Apply speed settings to motors!
void Drive();

// Adjust parameters based on max light measured value
void findDerThresholds(int method=0);

// print out light adjust data
void printDerThresholdsData();

// Give user feedback as to detection state via LEDs
void feedbackLights();

// determine racing mode
void useMode();
     
// capture log data
void captureData();

// dump log data to terminal
void dumpData();

void plotData(uint16_t* LineScanData, float* derivLineScanData);

// calculate std. deviation of derivative
// DH since using global variables do we need to pass things in via argument pointers?
void calcStdDevDeriv();
           
#endif
