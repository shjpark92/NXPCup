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
void MasterControlProgram();


// prints out line scan data for you
void printLineScanData(uint16_t *LineScanData);

// calculates derivative of line scan data
void derivativeLineScan(uint16_t* LineScanDataIn, float* DerivLineScanDataOut);

// prints out derivative of line scan data -- assumes deriv already calculated
void printDerivLineScanData(float* derivLineScanData);

// serves to grab a frame of camera data
void grabCameraFrame();

// find negative and positive edges in image data, store in array, combines edges found in adjacent pixels
void findEdges(float* derivLineScanData);

// improved algo to find negative and positive edges in image data, store in array, combines edges found in adjacent pixels
void findEdges_v2(float* derivLineScanData);

// prints out edge data found
void printEdgesFound();

// review edge data
//   - report line position if there
//   - set starting gate flag if there
//   - do nothing with position value otherwise
void reviewEdges();

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
void adjustLights();

// print out light adjust data
void printAdjustLightsData();

// Give user feedback as to detection state via LEDs
void feedbackLights();

// read DIP switches and potentiometers for setting changes
void readSwitches();
     
// capture log data
void captureData();

// dump log data to terminal
void dumpData();
           
#endif
