#ifndef INCLUDES
#define INCLUDES

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#include "definitions.h"
double Speed = 0.0;

double linesensor_black[] = {52.5, 	52, 57, 55.5, 	55, 54, 56, 57};
double linesensor_floor[] = {61, 	62, 68, 67, 	66, 63, 66, 65};
double linesensor_white[] = {82, 	88, 94, 95, 	93, 86, 90, 85};



enum
{
	mot_stop = 1,
	mot_move,
	mot_turn,
	mot_line,
	mot_followwall,
	mot_turnr,
	mot_drive

};

// Obstacles.
enum
{
	ms_obs1,
	ms_obs2,
	ms_obs3,
	ms_obs4,
	ms_obs5,
	ms_obs6,
	ms_end
};


enum end_conditions 
{
	end_dist,
	end_cross,
	end_black_line_found,
	end_ir
};

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
	double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;

int left_enc_old, right_enc_old;

typedef struct
{ //input
	int cmd;
	int curcmd;
	double speedcmd;
	double dist;
	double angle;
	double left_pos, right_pos;
	char direction;
	// parameters
	double w;
	//output
	double motorspeed_l, motorspeed_r;
	int finished;
	// internal variables
	double startpos;
	double radius;
	int end;
	int ir_index;
	double ir_dist;
	char side;
} motiontype;

typedef struct
{
	int state, oldstate;
	int time;
} smtype;

double visionpar[10];
double laserpar[10];
double ls_calib[8];

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('r'); i++)
		if (strcmp(tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('w'); i++)
		if (strcmp(tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}
/*****************************************
* odometry
*/
typedef struct
{ //input signals  // V

	int left_enc, right_enc; // encoderticks
	// parameters
	double w;	  // wheel separation
	double cr, cl; // meters per encodertick
				   //output signals
	double right_pos, left_pos;
	// internal variables
	int left_enc_old, right_enc_old;

	// odometry pose
	double x_pos, y_pos;
	double current_theta, old_theta, reference_theta;
} odotype;

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
smtype mission1;

/**********************************************************
 * 
 * 			GLOBAL VARIABLES USED AS END CONIDTIONS
 * 
 **********************************************************/
int crossing_black_line = 0;
int black_line_found = 0;










#endif
