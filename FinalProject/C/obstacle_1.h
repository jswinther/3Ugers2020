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
int raw2real(int raw);
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
            for(int i = 0; i < irsensor->length; ++i) {
                printf("IR Sensor [%d] = %d\n", i, raw2real(irsensor->data[i]));
		        if(obs1_n > 0) {
                    obs1 = obs1_turn;
                    obs1_n--;
                }
                else obs1 = obs1_end;
            }
            break;
        
        case obs1_turn:
            if(turn(1.57, 0.3, mission.time)) {
                obs1 = obs1_fwd;
            }
            break;

        case obs1_end:
            finished = 1;
	        break;
    }
    return finished;
}

int raw2real(int raw) {
    return 16/(raw - 76);
}

