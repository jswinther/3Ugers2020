#include "functions.h"

/**
 * Enums and Structs
 **/
enum {
    obs6_fwd,
    obs6_measure,
    obs6_end
};

int obs6 = obs6_fwd;
/**
 * Prototypes
 **/
int run_obstacle_6();


/**
 * Functions
 **/
int run_obstacle_6() {
    printf("%d\n", obs6);
    return 1;
}


