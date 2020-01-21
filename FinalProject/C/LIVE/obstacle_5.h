#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs5_fwd,
    obs5_measure,
    obs5_end
};

int obs5 = obs5_fwd;
/**
 * Prototypes
 **/
int run_obstacle_5();


/**
 * Functions
 **/
int run_obstacle_5() {
    printf("%d\n", obs5);
    return 1;
}


