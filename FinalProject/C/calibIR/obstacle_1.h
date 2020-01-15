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
/**
 * Prototypes
 **/
int run_obstacle_1();
double raw2real(double raw);
int laserparLength = (sizeof(laserpar)/sizeof(double))-1;
int obs1_n = 4;
/**
 * Functions
 **/
int run_obstacle_1() {
    int finished = 0;
    switch(obs1) {
        case obs1_fwd:
            if(fwd(0.80,0.3,mission.time)) {
                obs1 = obs1_measure;
            }
	        break;

        case obs1_measure:
            for(int i = 0; i < laserparLength; i++) {
                printf("\nIR Sensor [%d] = %f  /  %f", i, laserpar[i],raw2real(laserpar[i]));
		        if(obs1_n > 0) {
                    obs1 = obs1_turn;
                    obs1_n--;
                }
                else obs1 = obs1_end;
            }
            break;
        
        case obs1_turn:
            if(turn(1.57, 0.3, mission.time)) {
                printf("\nturn");
                obs1 = obs1_fwd;
            }
            break;

        case obs1_end:
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

