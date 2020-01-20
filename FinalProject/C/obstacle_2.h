#include "functions.h"

/**
 * Enums and Structs
 **/
enum Obstacle2 {
    obs2_fl, //follow black line left until it finds crossing black line.
    obs2_fm1, 
    obs2_fwd, //Drives just past the black cross.
    obs2_fm2, //Follow line and push box forward until it hits cross.
    obs2_fwd2, //Drive past line.
    obs2_drive_back, //
    obs2_end
};

/**
 * Prototypes
 **/
int run_obstacle_2();

int obs2_initFlag=0;

/**
 * Functions
 **/
int run_obstacle_2() {
    if(!obs2_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state=obs2_fl;
        mission.oldstate=-1;
        obs2_initFlag = 1;
        //printf("\nStart position x,y: %f, %f", odo.x,odo.y); 
    }

    int finished = 0;
    sm_update(&mission); 
    //printf("\nMission state: %d",mission.state);
    //printf("\nMission time: %d",mission.time);
    switch(mission.state) 
    {
        case obs2_fl:
            if(fl(end_dist, 0.7, 0, 0, 0.3, mission.time, 'l')) 
            {
                mission.state = obs2_fm1;
            }
	        break;
        case obs2_fm1:
            if (fl(end_cross, 20, 0, 0, 0.6, mission.time, 'm'))
            {   
                mission.state = obs2_drive_back;
            }
            
            break;
        case obs2_drive_back:
            if(fwd(-1, -0.3, mission.time))
            {
                mission.state = obs2_end;
            }
            break;
        case obs2_end:
            finished = 1;
	        break;
    }
    
    return finished;
}


