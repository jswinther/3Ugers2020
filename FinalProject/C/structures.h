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

enum 
{
   mot_stop=1,
   mot_move,
   mot_turn
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

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct 
{
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;

typedef struct
{ //input signals
	int left_enc,right_enc; // encoderticks
	// parameters
	double w;	// wheel separation
	double cr,cl;   // meters per encodertick
   	//output signals
	double right_pos,left_pos;
	// internal variables
	int left_enc_old, right_enc_old;
  //odo
  double x, y, theta;
} odotype;

typedef struct
{ //input
   int cmd;
   int curcmd;
	double speedcmd;
	double dist;
	double angle;
	double left_pos,right_pos;
	// parameters
	double w;
	//output
	double motorspeed_l,motorspeed_r; 
	int finished;
	// internal variables
	double startpos;
} motiontype;
	       
typedef struct
{
   int state,oldstate;
	int time;
} smtype;

double visionpar[10];
double laserpar[10];



componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/



// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
smtype mission1;

#endif
