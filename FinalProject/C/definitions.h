#include "math.h"


/**
 * Forward and Turn speed.
 **/
#define DEF_FWD 0.3
#define DEF_TURN 0.3

/**
 * Simulation = 0, Real World = 1
 **/
#define ROBOT 1

#if ROBOT
    #define ROBOTPORT 24902 
#else
    #define ROBOTPORT 8000
#endif


/**
 * 8000 = Simulation, 24902 = Robot
 **/


/**
 * Robot specific measurements.
 **/
#define WHEEL_DIAMETER   0.06522
#define WHEEL_SEPARATION 0.26
#define ROBOT_WIDTH 0.261
#define ROBOT_LENGTH 0.235 //Length of orego between the wheels and to the IR sensors.
#define ROBOT_HEIGHT 0.43

#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
/**
 * The raw value from linesensors on concrete floor.
 * Choose the lowest measured value
 * The raw value from linesenors on black tape
 * Choose the highest measured value
 **/

//real World
//#define lineBlack 43-3      //43
//#define lineWhite 74+3      //74
// Simulation
#define lineBlack 0
#define lineWhite 128