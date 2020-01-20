#include "functions.h"

/**
 * Enums and Structs
 **/
enum Obstacle2 {
    obs2_fl, //follow black line left until it finds crossing black line.
    obs2_drive, //drive until black line 
    obs2_fm1, 
    obs2_fwd1, //Drives just past the black cross. 
    obs2_bwd,   // drives 1 meter back
    obs2_turn, // turn -90 deg
    obs2_turnr, // turns wiht radius
    obs2_fl_cross,
    obs2_fl_cross_2,
    obs2_fwd2,
    obs2_end
};

/**
 * Prototypes
 **/
int run_obstacle_2();

int obs2_initFlag=0;
int obs2_n=3;
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
                mission.state = obs2_fwd1;
            }
            
            break;
        case obs2_fwd1:
            if (fwd(0.12,0.3,mission.time))
            {   

                mission.state = obs2_bwd;
            }
            
            break;
        case obs2_bwd:
            if (fwd(1,-0.3,mission.time))
            {   
                mission.state = obs2_turn;
            }
            break;
        
        case obs2_turn:
            if (turn(-1.57,0.3,mission.time))
            {   
                mission.state = obs2_drive;
              //  printf("\n\n\n\ndrive done");
            }
            break;
        case obs2_drive:
            if (drive(end_black_line_found,0.3,mission.time))
            {   
                //printf("\ndrive done");
                mission.state = obs2_turnr;
            }
            break;

        case obs2_turnr:
            if (turnr(0.2,1.57,0.3,mission.time))
            {   
                printf("\nturnr done");
                mission.state = obs2_fl_cross;
            }
            break;
        case obs2_fl_cross:
            if(fl(end_cross, 20, 0, 0, 0.6, mission.time, 'm'))
            {
                mission.state = obs2_fwd2;
            }
            break;
        case obs2_fwd2:
            if(fwd(0.12, 0.6, mission.time))
            {
                mission.state = obs2_fl_cross_2;
            }
            break;
        case obs2_fl_cross_2:
            if(fl(end_cross, 20, 0, 0, 0.6, mission.time, 'm'))
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


