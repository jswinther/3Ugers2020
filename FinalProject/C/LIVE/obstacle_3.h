#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs3_fwd,
    obs3_fl_laser,
    obs3_fl_dist,
    obs3_turn,
    obs3_drive_through_gate,
    obs3_end
};

int obs3 = obs3_fwd;
/**
 * Prototypes
 **/
int run_obstacle_3();

double obs3_dist = 0;

int obs3_initFlag=0;
int obs3_n=3;
/**
 * Functions
 **/
int run_obstacle_3() {
    if(!obs3_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state=obs3_fwd;
        mission.oldstate=-1;
        obs3_initFlag = 1;
        //printf("\nStart position x,y: %f, %f", odo.x,odo.y); 
    }

    int finished = 0;
    sm_update(&mission); 
    //printf("\nMission state: %d",mission.state);
    //printf("\nMission time: %d",mission.time);
    switch(mission.state) 
    {
        case obs3_fwd:
            if(fwd(0.5, 0.6, mission.time))
            {
                mission.state = obs3_fl_laser;
            }
            break;
        case obs3_fl_laser:
            if(fl(end_ir, 20, 0, 1.5, 0.6, mission.time, 'm'))
            {
                obs3_dist = laserpar[0]*sin(40) + 0.05;
                mission.state = obs3_fl_dist;
            }
            break;
        case obs3_fl_dist:
            if(fl(end_dist, obs3_dist, 0, 0, 0.6, mission.time, 'm'))
            {
                mission.state = obs3_turn;
            }
            break;
        case obs3_turn:
            if(turn(1.57, 0.3, mission.time))
            {
                mission.state = obs3_drive_through_gate;
            }
            break;
        case obs3_drive_through_gate:
            if(fwd(1.5, 0.3, mission.time))
            {
                mission.state = obs3_end;
            }
            break;
        case obs3_end:
            finished = 1;
	        break;
    }
    
    return finished;
}

