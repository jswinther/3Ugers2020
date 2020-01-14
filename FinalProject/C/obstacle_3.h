#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs3_fwd,
    obs3_measure,
    obs3_end
};

int obs3 = obs3_fwd;
/**
 * Prototypes
 **/
int run_obstacle_3();


/**
 * Functions
 **/
int run_obstacle_3() {
    printf("%d\n", obs3);
    return 1;
}


