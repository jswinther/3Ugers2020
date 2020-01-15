#include "functions.h"
 
/**
 * Enums and Structs
 **/
enum {
    IRc_IRmeasure,
    IRc_fwd,
    IRc_end
};

//mission.state = IRc_IRmeasure;
/**
 * Prototypes
 **/
int IRcalibration();
double raw2real(double raw);
void sm_update(smtype *p);

int laserparLength = (sizeof(laserpar)/sizeof(double))-1;
int IRc_n = 6;

double laserparCal[6];

int measurecount = 100;

smtype mission;

int flag_init = 0;


/**
 * Functions
 **/
int IRcalibration() {
    if(!flag_init)
    {
        mission.state=IRc_IRmeasure;
        mission.oldstate=-1;
        flag_init = 1;
    }


    //printf("\nMission time: %d",mission.time);
    int finished = 0;
    //printf("\nstate = %d",mission.state);

    sm_update(&mission); 
    switch(mission.state) {
        case IRc_IRmeasure:
           // printf("\nmodulus: %d", mission.time % 17);
            if(mission.time % 22 == 0)
            {
                printf("\nIRMeasure");

            // printf("\nGO - case 1");
                if (measurecount == 1)
                {
                //    printf("\nIF");
                //   printf("\ngot to fwd");
                    measurecount = 100;
                    mission.state = IRc_fwd;
                    laserparCal[IRc_n-1] = laserparCal[IRc_n-1]/99;
                    IRc_n--;

                }
                else if (IRc_n < 1)
                {
               //     printf("\n!!! RUNED END !!! IRC_n: %d",IRc_n);
                //   printf("\nELSE IF");
                    mission.state = IRc_end;
                // printf("\ngot to END!");
                }
                
                else
                {
                //    printf("\nmeasurement nr: %d",measurecount);
                    measurecount--;
                    laserparCal[IRc_n-1] +=laserpar[4]; // goes from 6 to 1
                 //   printf("\n$$ $$ Laserpar: %f ",laserpar[4]);
                    //printf("\nELSE");
                }
            }
            break;

        case IRc_fwd:
           // printf("\nGO - case 2");
            if(fwd(0.1,0.3,mission.time)) {
        //        printf("\n                number of runs: %d",IRc_n);
                mission.state = IRc_IRmeasure;
                
                
                
            }

            break;

        case IRc_end:
           // printf("\nGO - case 3");
            finished = 1;
            printf("\nFinal results:\n");
            for (int i = 0; i < 6;i++ )
            {
                printf("%f ",laserparCal[i]);
            }
	        break;
    }
    return finished;
}

double raw2real(double raw) {
    if(raw == 1000)
        return -1;
    else
        return 16/(raw - 76);
}

void sm_update(smtype *p)
{
    if (p->state!=p->oldstate)
    {
       p->time=0;
       p->oldstate=p->state;
    }
    else 
    {
     p->time++;
    }
}