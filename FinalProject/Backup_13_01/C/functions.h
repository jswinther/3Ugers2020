#include "structures.h"

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);


void reset_odo(odotype *p);
void update_odo(odotype *p);


void update_motcon(motiontype *p);	       
int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
void sm_update(smtype *p);






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