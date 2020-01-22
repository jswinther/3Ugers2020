#include "math.h"

#ifndef DEFINITIONS
#define DEFINITIONS
/**
 * Forward and Turn speed.
 **/
#define DEF_FWD 0.3
#define DEF_TURN 0.3

/**
 * Simulation = 0, Real World = 1
 **/
#define ROBOT 0

#if ROBOT
    #define ROBOTPORT 24902
    #define K_MOVE 0.5
    #define K_LINE 0.2
    double ls_b[] = {50,    50,     50,     50,     50,     50,     50,     50 }; // Black Tape
    double ls_w[] = {90,    90,     90,     90,     90,     90,     90,     90 }; // White Tape
#else
    #define ROBOTPORT 8000
    #define K_MOVE 0.5
    #define K_LINE 0.2
    double ls_b[] = {0,     0,      0,      0,      0,      0,      0,      0  };
    double ls_w[] = {128,   128,    128,    128,    128,    128,    128,    128};
#endif

/**
 * Robot specific measurements.
 **/
#define WHEEL_DIAMETER   0.06522
#define WHEEL_SEPARATION 0.26
#define ROBOT_WIDTH 0.261
#define ROBOT_LENGTH 0.235 //Length of orego between the wheels and to the IR sensors.
#define ROBOT_HEIGHT 0.43

#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#endif