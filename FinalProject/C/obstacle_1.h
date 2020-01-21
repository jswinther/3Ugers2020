#include "functions.h"

/**
 * Enums and Structs
 **/
enum OBSTACLE_1_STATES
{
    obs1_fl, /* Follow black line until it has driven 1 meter, such that it is facing the box we want to measure distance to. */
    obs1_measure, /* Measure the distance 10 times from where the robot is and add this to the odox and print out ths distance. */
    obs1_end /* The obstacle is over and it goes to obstacle 2. */
};

/**************\
 * Prototypes *
\**************/
int run_obstacle_1();
int laserparLength = (sizeof(laserpar) / sizeof(double)) - 1;
int obs1_n = 10;
int obs1_initFlag = 0;

double distance = 0;

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

/*************\
 * Functions *
\*************/
int run_obstacle_1()
{

    if (!obs1_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state = obs1_fl;
        mission.oldstate = -1;
        obs1_initFlag = 1;
    }

    int finished = 0;
    sm_update(&mission);
    //printf("\nMission state: %d",mission.state);
    //printf("\nMission time: %d",mission.time);
    switch (mission.state)
    {
    case obs1_fl:
        if (fl(end_dist, 1.20, 0, 0, 0.5, mission.time, 'm'))
        {
            mission.state = obs1_measure;
        }
        break;

    case obs1_measure:
        if (mission.time % 25 == 0) // to insure that the laserpar array has been updated
        {
            distance += laserpar[4];
            //printf("\nIR Sensor = %f", laserpar[4]);

            if (obs1_n > 1)
            {
                obs1_n--;
            }
            else
            {
                distance = (distance / 10) + (odo.x_pos + 0.235); //0.235 distance from origo to sensor
                printf("\ndistance to box is = %f", distance);
                mission.state = obs1_end;
            }
        }
        break;

    case obs1_end:
        finished = 1;
        break;
    }
    return finished;
}
