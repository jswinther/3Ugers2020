/*
 * An example SMR program.
 *
 */
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

#define DEF_FWD 0.3
#define DEF_TURN 0.6

//exercise 3.1
struct sample_31{
	int time;
	double x;
	double y;
        double theta;
};

struct sample_31 samp;
struct sample_31 log_31[1000];
int count_31 = 0;
int sampleRate = 2;

int missionTime = 0;

FILE *linefp;

double globalSpeedFwd = DEF_FWD;
double globalSpeedTurn = DEF_TURN;

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);









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
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.26	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902 //8000


typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		//pos
		double x,y,current_theta;
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

typedef struct{//input
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
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p);	       

/**
 * Usermade functions
 * **/
void follow_the_line_1(motiontype *p, symTableElement *s);
void center_of_mass(motiontype *p, symTableElement *s);

int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);



typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end};

int main()
{
  linefp = fopen("log_3_4.dat", "w");
  int running,n=0,arg,time=0;
  double dist=0,angle=0;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);

      

     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=0.256;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
running=1; 
mission.state=ms_init;
mission.oldstate=-1;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_odo(&odo);
  
/****************************************
/ mission statemachine   
*/
  sm_update(&mission);
  if(missionTime % 100 == 0)
    fprintf(linefp,"%d, %f, %f, %f\n",(int)missionTime/100, odo.x,odo.y,odo.theta);

        
  
  
   if(count_31 < 1000 && (missionTime % sampleRate) == 0){
   	samp.time = missionTime;
   	samp.x = odo.x;
   	samp.y = odo.y;
 	samp.theta = odo.theta;
   	log_31[count_31] = samp;
   	count_31++;
   }
   missionTime++;


   switch (mission.state) {
     case ms_init:
       n=4; dist=0;angle=90.0/180*M_PI;
       fwd(dist,globalSpeedFwd,mission.time);
       mission.state= ms_fwd;      
     break;
  
     case ms_fwd:
  follow_the_line_1(&mot, linesensor);
	if(missionTime < 2000) 
    mission.state=ms_fwd;
  else
    mission.state=ms_end;
	
     break;
  
     case ms_turn:
       if (turn(angle,globalSpeedTurn,mission.time)){
         n=n-1;
	 if (n==0) 
	   mission.state=ms_end;
	 else{
	   globalSpeedFwd = DEF_FWD;
	   mission.state=ms_fwd;
	 }
	}
     break;    
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }  
/*  end of mission  */
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot);
  follow_the_line_1(&mot,linesensor);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;


   
}/* end of main control loop */
  
FILE *f;
char outfile[] = "log.dat";


f = fopen(outfile, "w");
for(int i = 0; i < missionTime/sampleRate; i++){
 	if(log_31 == NULL){break;}
	fprintf(f, "%d %f %f %f\n", log_31[i].time, log_31[i].x, log_31[i].y, log_31[i].theta);
}
fclose(f);
fclose(linefp);

  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;

  p->x = p->y = p->theta = 0;
}

void update_odo(odotype *p)
{
  int delta;
  double Ur, Ul, U, delTheta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  Ur=delta * p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  Ul=delta * p->cl;

  U = (Ur + Ul)/2;
  delTheta = (Ur - Ul)/WHEEL_SEPARATION;

  p->theta = p->theta + delTheta;
  p->x = p->x + U*cos(p->theta);
  p->y = p->y + U*sin(p->theta);
  
}


void update_motcon(motiontype *p){ 

double distLeft = fabs((p->dist - (p->right_pos+p->left_pos)/2-p->startpos));
double angleLeft = fabs((p->angle*p->w)/2 - (p->right_pos-p->startpos));
double K = 5;
double K1 = DEF_TURN*(0.00625);
double accel = fabs(K * (p->dist - p->right_pos-p->startpos));
double rotAccel = fabs(K1 * (angleLeft));
double vMax = sqrt(2*accel*distLeft);
double rMax = sqrt(2*rotAccel*angleLeft);

// || fabs(distLeft) < 0.15


if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
       p->startpos=(p->left_pos+p->right_pos)/2;
       p->curcmd=mot_move;
       break;
       
       case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     case mot_move:
        
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }	  
       else {
	
	
	if(vMax+0.05 < p->motorspeed_l){

	  globalSpeedFwd -= accel;
	}else if(p->motorspeed_l < p->speedcmd - accel){
	  globalSpeedFwd += accel;
	}
	  p->motorspeed_l=globalSpeedFwd;
          p->motorspeed_r=globalSpeedFwd; 
	
       }
     break;
     
     case mot_turn:
//printf("motturn\n");
       if (p->angle>0){

	  if(rMax < p->motorspeed_l && p->right_pos-p->startpos < (p->angle*p->w)/2){
	      globalSpeedTurn += rotAccel;
	      p->motorspeed_r= globalSpeedTurn;
	      p->motorspeed_l= -globalSpeedTurn;
	  }else if(rMax > p->motorspeed_l && p->right_pos-p->startpos < (p->angle*p->w)/2){
	      globalSpeedTurn -= rotAccel;
	      p->motorspeed_r= globalSpeedTurn;
	      p->motorspeed_l= -globalSpeedTurn;
	  }
	  else {	     
            p->motorspeed_r=0;
	    p->motorspeed_l=0;
            p->finished=1;
	  }
	}
	else {
	  if (p->left_pos-p->startpos < (fabs(p->angle)*p->w)/2){
	      p->motorspeed_r-= rotAccel;
	      p->motorspeed_l+= rotAccel;
	  }
	  else {	     
            p->motorspeed_l=0;
	    p->motorspeed_r=0;
            p->finished=1;
	  }
	}
 
     break;
   }   
}   


int fwd(double dist, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_move;
     mot.speedcmd=speed;
     mot.dist=dist;
     return 0;
   }
   else
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}

void follow_the_line_1(motiontype *p, symTableElement *s) {
  
  int center = 3;
  int index_min = 0;
  int min = 500;

  for (int i = 0; i < s->length; i++)
  {
    if(s->data[i] < min) {
      min = s->data[i];
      index_min = i;
    }
  }
  int offset_from_center = center - index_min;
  if(offset_from_center >= -1 && offset_from_center <= 2) {
    printf("Center\n");
    p->motorspeed_l = DEF_FWD;
    p->motorspeed_r = DEF_FWD;
  } else if(offset_from_center < -1) {
    printf("Dec Right\n");
    // Deccelerate right motor
    p->motorspeed_l -= offset_from_center/10 * 0.005;
    p->motorspeed_r = DEF_FWD; 
  } else if(offset_from_center > 2) {
    printf("Dec Left\n");
    // Deccelerate left motor
    p->motorspeed_r -= offset_from_center/10 * 0.005;
    p->motorspeed_l = DEF_FWD; 
  }
  if(offset_from_center == 0.000 && missionTime > 500) {
    mission.state = ms_end;
  }
}

void center_of_mass(motiontype *p, symTableElement *s) {
  
  double sum_xl = 0;
  double sum_l = 0;

  for (int i = 0; i < s->length; i++)
  {
    sum_xl += (1 - (s->data[i]/128)) * odo.x;
  }
  
  for (int i = 0; i < s->length; i++)
  {
    sum_l += (s->data[i]/128);
  }

  double x_c = sum_xl/sum_l;
  double offset_from_center = x_c;
  printf("x_c = %f\n", x_c);
  
  if(offset_from_center >= -2 && offset_from_center <= 2) {
    p->motorspeed_l = DEF_FWD;
    p->motorspeed_r = DEF_FWD;
  } else if(offset_from_center < -2) {
    printf("Dec Right\n");
    // Deccelerate right motor
    p->motorspeed_l -= offset_from_center/10 * 0.005;
    p->motorspeed_r = DEF_FWD; 
  } else if(offset_from_center > 2) {
    printf("Dec Left\n");
    // Deccelerate left motor
    p->motorspeed_r -= offset_from_center/10 * 0.005;
    p->motorspeed_l = DEF_FWD; 
  }
  if(x_c == 0.000 || missionTime > 500) {
    mission.state = ms_end;
  }

}



