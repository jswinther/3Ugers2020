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
int followline(char type);
int followwall(char side, double dist);
int resetmotors();
int ignoreobstacles();
int targethere();

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
}

void update_odo(odotype *p)
{
    int delta;

    delta = p->right_enc - p->right_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->right_enc_old = p->right_enc;
    p->right_pos += delta * p->cr;
  
    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta * p->cl;
}


void update_motcon(motiontype *p)
{ 
    if (p->cmd !=0)
    {
        p->finished=0;
        switch (p->cmd)
        {
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
   
    switch (p->curcmd)
    {
        case mot_stop:
            p->motorspeed_l=0;
            p->motorspeed_r=0;
            break;
        case mot_move:
            if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist)
            {
                p->finished=1;
	            p->motorspeed_l=0;
                p->motorspeed_r=0;
            }	  
            else 
            {	
	            p->motorspeed_l=p->speedcmd;
                p->motorspeed_r=p->speedcmd;
            }
            break;  
        case mot_turn:
            if (p->angle>0)
            {
                p->motorspeed_l=0;
	            if (p->right_pos-p->startpos < p->angle*p->w)
                {
	                p->motorspeed_r=p->speedcmd;
	            }
	            else 
                {	     
                    p->motorspeed_r=0;
                    p->finished=1;
	            }
	        }
	        else 
            {
                p->motorspeed_r=0;
	            if (p->left_pos-p->startpos < fabs(p->angle)*p->w) 
                {
                    p->motorspeed_l=p->speedcmd;
                }
	            else 
                {	     
                    p->motorspeed_l=0;
                    p->finished=1;
	            }
	        }
            break;
    }   
}   


int fwd(double dist, double speed,int time)
{
    if (time==0){ 
        mot.cmd=mot_move;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
    {
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

int followline(char type)
{
    if (type == 'm')
    {

    }
    else if (tyoe == 'r')
    {
        /* code */
    }
    else if (type == 'l')
    {
        /* code */
    }
    
    
    printf("Not yet implemented followline");
    return 1;
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
    printf("Not yet implemented stop");
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
