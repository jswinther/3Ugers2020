#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs2_fwd,
    obs2_measure,
    obs2_end
};

int obs2 = obs2_fwd;
/**
 * Prototypes
 **/
int run_obstacle_2();


/**
 * Functions
 **/
int run_obstacle_2() {
    printf("%d\n", obs2);
    return 1;
}


