#include "structures.h"


#ifndef FUNCTIONS
#define FUNCTIONS

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);


void reset_odo(odotype *p);
void update_odo(odotype *p);


void update_motcon(motiontype *p);	      
void sm_update(smtype *p);


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int drive(double speed);
int turnr(double radius, double degrees);
int stop();
int idle();
int followline(double speed, char type);
int crossingblackline();
int followwall(char side, double dist);
int resetmotors();
int ignoreobstacles();
int targethere();
void center_of_mass(motiontype *p, symTableElement *s);
/**
 * SMR-CL Methods.
 **/

/**
 * turnr
 * followline
 * idle
 * drive
 * followwall
 * resetmotors
 * ignore obstacles
 * obstacle detection
 * targethere
 **/


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

    // Update Ur
    Ur = delta * p->cr;




  
    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta * p->cl;

    // Update Ul
    Ul = delta * p->cl;

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

int fwd(double dist, double speed,int time)
{
    if (time==0){ 
        printf("\nfwd speed 1: %f ",mot.speedcmd);
        mot.cmd=mot_move;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
    {
        printf("\nfwd speed 2: %f ",mot.speedcmd);
        return mot.finished;
    }
}

int turn(double angle, double speed,int time)
{
    if (time==0)
    { 
        mot.cmd=mot_turn;
        mot.speedcmd=speed;
        mot.angle=angle;
        return 0;
    }
    else
    {
        return mot.finished;
    }
}


void sm_update(smtype *p)
{
    if (p->state!=p->oldstate)
    {
       p->time=0;
       p->oldstate=p->state;
    }
    else 
    {
     p->time++;
    }
}

/**
 * See Exercise 7, 3.4 centre of mass algorithm.
 * \param type is 'l' for left tracking, 'm' for middle tracking, 'r' for right tracking.
 * \return returns 1 if there is a blackline returns 0 when there is no more blackline.
 **/
int followline(double speed, char type)
{
        double xl = 0;
        double l = 0;
        double sensor_offset[] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
        double offset = 0;
        switch(type) 
        {
        case 'l':
            offset = 2;    
            break;
        case 'm':
            offset = 0;
            break;
        case 'r':
            offset = -2;
            break;
        }

        
        for (int i = 0; i < linesensor->length; i++)
        {
            xl += (1 - linesensor->data[i]/128) * (sensor_offset[i] + offset);
        }

        for (int i = 0; i < linesensor->length; i++)
        {
            l += (1 - linesensor->data[i]/128);
        }

        if(l == 0) {
            switch(type) {
                case 'm':
                    mot.motorspeed_l = 0;
                    mot.motorspeed_r = 0;
                    break;
                case 'l':
                    mot.motorspeed_l = speed;
                    mot.motorspeed_r = 0;
                    break;
                case 'r':
                    mot.motorspeed_l = 0;
                    mot.motorspeed_r = speed;
                    break;
            }
        }


        double black_line_center = xl/l;
       // printf("Odo x = %f\n", odo.x);
        //printf("Center of the black line is %.2f\n", black_line_center);

       if(1 > black_line_center + 0.05 && -1 < black_line_center - 0.05) {
                mot.motorspeed_l = speed;
                mot.motorspeed_r = speed;
            } else if (1 < black_line_center + 0.05) {
                mot.motorspeed_l -= 0.05;
                mot.motorspeed_r = speed;
            } else if(-1 > black_line_center - 0.05) {
                mot.motorspeed_l = speed;
                mot.motorspeed_r -= 0.05;
            } else {
                return mot.finished;
            }
    return 0;
}

int crossingblackline() {
    double sum = 0;
    for (int i = 0; i < 9; i++)
    {
        sum += linesensor->data[i];
    }
    return sum < BLACK_TAPE*9 ? 0 : 1;
}

int followwall(char side, double dist)
{
    printf("Not yet implemented followwall");
    return 1;
}


int drive(double speed)
{
    printf("Not yet implemented drive");
    return 1;
}
int turnr(double radius, double degrees)
{
    printf("Not yet implemented turnr");
    return 1;
}
int stop()
{
    printf("STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP ANHOLD");
    mot.motorspeed_l = 0;
    mot.motorspeed_r = 0;
    //printf("Not yet implemented stop");
    return 1;
}
int idle()
{
    printf("Not yet implemented idle");
    return 1;
}

int resetmotors()
{
    printf("Not yet implemented resetmotors");
    return 1;
}
int ignoreobstacles()
{
    printf("Not yet implemented ignore obstacles");
    return 1;
}
int targethere()
{
    printf("Not yet implemented targethere");
    return 1;
}



#endif
