#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs2_fl,
    obs2_measure,
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
            if(fl()) 
            {
                mission.state = obs2_measure;
            }
            
	        break;

        case obs2_measure:
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


