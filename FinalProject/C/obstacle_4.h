#include "functions.h"

/**
 * Enums and Structs
 **/
enum
{
    obs4_fwd, // not used
    obs4_drive,
    obs4_turn,
    obs4_flw,
    obs4_fwd2,
    obs4_turn2,
    obs4_fwd3,
    obs4_turn3,
    obs4_flw2,
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
    if (!obs4_initFlag) // insures that the statemachine starts in the correct case
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
        if (fwd(0.5, 0.3, mission.time))
        {
            printf("obs4_fwd\n");
            mission.state = obs4_drive;
        }
        break;
    case obs4_drive:
        if (drive(end_ir, 0.3, mission.time))
        {
            printf("obs4_drive\n");
            mission.state = obs4_turn;
        }
        break;

    case obs4_turn:
        if (turn(1.57, 0.3, mission.time))
        {
            printf("\nturn done");
            mission.state = obs4_flw;
            
        }
        break;

    case obs4_flw:
        if (followwall(mission.time,0.2))
        {
            printf("\nfollow wall done");
            mission.state = obs4_fwd2;
        }
        break;
    
    case obs4_fwd2:
        if (fwd(0.48, 0.3, mission.time))
        {
            printf("obs4_fwd2\n");
            mission.state = obs4_turn2;
        }
        break;
        
    case obs4_turn2:
        if (turn(-1.57, 0.3, mission.time))
        {
            printf("obs4_turn2\n");
            mission.state = obs4_fwd3;
        }
        break;

    case obs4_fwd3:
        if (fwd(0.7, 0.3, mission.time))
        {
            printf("obs4_fwd3\n");
            mission.state = obs4_turn3;
        }
        break;
        
    case obs4_turn3:
        if (turn(-1.57, 0.3, mission.time))
        {
            printf("obs4_turn3\n");
            mission.state = obs4_flw2;
        }
        break;


    case obs4_flw2:
        if (followwall(mission.time,0.2))
        {
            printf("obs4_flw2\n");
            mission.state = obs4_end;
            //mission.state = obs4_fwd;
        }
        break;

    case obs4_end:
        finished = 1;
        break;
    }
    //printf("finished = %d\n", finished);
    return finished;
}
