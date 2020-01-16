#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs1_fwd,
    obs1_measure,
    obs1_turn,
    obs1_end
};

int obs1 = obs1_fwd;

/**************\
 * Prototypes *
\**************/
int run_obstacle_1();
double raw2real(double raw);
int laserparLength = (sizeof(laserpar)/sizeof(double))-1;
int obs1_n = 4;
int obs1_initFlag=0;

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

/*************\
 * Functions *
\*************/
int run_obstacle_1() {

    if(!obs1_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state=obs1_fwd;
        mission.oldstate=-1;
        obs1_initFlag = 1;
    }

    int finished = 0;
    sm_update(&mission); 
    printf("\nMission state: %d",mission.state);
    printf("\nMission time: %d",mission.time);
    switch(mission.state) 
    {
        case obs1_fwd:
            if(fwd(0.80,0.3,mission.time)) 
            {
                mission.state = obs1_measure;
            }
	        break;

        case obs1_measure:
            if(mission.time % 22 == 0) // to insure that the laserpar array has been updated
            {
                for(int i = 0; i < laserparLength; i++) 
                {
                    printf("\nIR Sensor [%d] = %f", i, laserpar[i]);
                }

                if(obs1_n > 0) 
                {
                    mission.state = obs1_turn;
                    obs1_n--;
                }
                else 
                {
                    mission.state = obs1_end;
                }
                
            }
            break;
        
        case obs1_turn:

            if(turn(1.57, 0.3, mission.time)) 
            {
                mission.state = obs1_fwd;
            }
            break;

        case obs1_end:
            finished = 1;
	        break;
    }
    return finished;
}

