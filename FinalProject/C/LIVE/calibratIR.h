#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    IRc_fwd,
    IRc_IRmeasure,
    IRc_end
};

int IRc = IRc_IRmeasure;
/**
 * Prototypes
 **/
int IRcalibration();
double raw2real(double raw);

int laserparLength = (sizeof(laserpar)/sizeof(double))-1;
int IRc_n = 6;

double laserparCal[6];

int measurecount = 10;

/**
 * Functions
 **/
int IRcalibration() {
    int finished = 0;
    switch(IRc) {
        case IRc_IRmeasure:

            if (!measurecount)
            {
                measurecount = 10;
                IRc = IRc_fwd;
                laserparCal[IRc_n] /= 10;
                IRc_n--;

            }
            else if (IRc_n > 1)
            {
                IRc = IRc_end;
            }
            
            else
            {
                printf("measurement nr: %d",measurecount);
                measurecount--;
                laserparCal[IRc_n] +=laserpar[4]; // goes from 6 to 1
            }
            break;

        case IRc_fwd:
            if(fwd(0.1,0.3,mission.time)) {
                printf("number of runs: %d",IRc_n);
                IRc = IRc_IRmeasure;
                
            }

            break;

        case IRc_end:
            finished = 1;
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

