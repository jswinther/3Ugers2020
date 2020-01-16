#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs1_fl,
    obs1_measure,
    obs1_end
};

/**************\
 * Prototypes *
\**************/
int run_obstacle_1();
double raw2real(double raw);
int laserparLength = (sizeof(laserpar)/sizeof(double))-1;
int obs1_n = 10;
int obs1_initFlag=0;

double distance =0;

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

/*************\
 * Functions *
\*************/
int run_obstacle_1() {

    if(!obs1_initFlag) // insures that the statemachine starts in the correct case
    {
        mission.state=obs1_fl;
        mission.oldstate=-1;
        obs1_initFlag = 1;
        printf("\nStart position x,y: %f, %f", odo.x,odo.y); 
    }

    int finished = 0;
    sm_update(&mission); 
    //printf("\nMission state: %d",mission.state);
    //printf("\nMission time: %d",mission.time);
    switch(mission.state) 
    {
        case obs1_fl:
            if(odo.theta > -1.43) 
            {
                followline(0.3, 'm');
                printf("\nangle postion theta: %f", odo.theta);  
                
            }
            else
            {
                stop();
                printf("\nEnd position x,y: %f, %f", odo.x,odo.y);  
                mission.state = obs1_measure;
            }
	        break;

        case obs1_measure:
            if(mission.time % 22 == 0) // to insure that the laserpar array has been updated
            {
                distance += laserpar[4];
                printf("\nIR Sensor = %f", laserpar[4]);                

                if(obs1_n > 1) 
                {
                    obs1_n--;
                }
                else 
                {
                    distance = (distance/10)+(odo.x+0.235);//0.235 distance from origo to sensor
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

