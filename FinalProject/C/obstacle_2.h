#include "functions.h"

/**
 * Enums and Structs
 **/
enum Obstacle2 {
    obs2_fl, //follow black line left until it finds crossing black line.
    obs2_fwd, //Drives just past the black cross.
    obs2_fm, //Follow line and push box forward until it hits cross.
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
            if(1) 
            {
                mission.state = obs2_fwd;
            }
            
	        break;

        case obs2_fwd:
            if(mission.time % 22 == 0) // to insure that the laserpar array has been updated
            {
                if (1)
                {
                    mission.state = obs2_end;
                }
                
            }
            break;
        
        case obs2_end:
            finished = 1;
	        break;
    }
    return finished;
}


