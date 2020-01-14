#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs4_fwd,
    obs4_measure,
    obs4_end
};

int obs4 = obs4_fwd;
/**
 * Prototypes
 **/
int run_obstacle_4();


/**
 * Functions
 **/
int run_obstacle_4() {
    printf("%d\n", obs4);
    return 1;
}


