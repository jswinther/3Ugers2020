#include "functions.h"

/**
 * Enums and Structs
 **/
enum
{
    obs4_fwd,
    obs4_turn,
    obs4_flw,
    obs4_end
};

int obs4 = obs4_fwd;
/**
 * Prototypes
 **/
int run_obstacle_4();
int obs4_initFlag = 0;

/**
 * Functions
 **/
int run_obstacle_4()
{
    if (!obs3_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state = obs4_fwd;
        mission.oldstate = -1;
        obs3_initFlag = 1;
        //printf("\nStart position x,y: %f, %f", odo.x,odo.y);
    }

    int finished = 0;
    sm_update(&mission);
    //printf("\nMission state: %d",mission.state);
    //printf("\nMission time: %d",mission.time);
    switch (mission.state)
    {
    case obs4_fwd:
        if (fwd(0.5, 0.6, mission.time))
        {
            mission.state = obs4_turn;
        }
        break;

    case obs4_turn:
        if (turn(1.57, 0.3, mission.time))
        {
            mission.state = obs4_flw;
        }
        break;

    case obs4_flw:
        if (followwall(0.5, 0.6, mission.time))
        {
            mission.state = obs4_end;
        }
        break;
    case obs4_fwd:
        if (fwd(0.5, 0.6, mission.time))
        {
            mission.state = obs4_end;
        }
        break;
    case obs4_fwd:
        if (fwd(0.5, 0.6, mission.time))
        {
            mission.state = obs4_end;
        }
        break;

    case obs4_end:
        finished = 1;
        break;
    }

    return finished;
}
