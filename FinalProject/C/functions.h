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
int fl(double dist, double speed,int time,char direction);

int drive(double speed);
int turnr(double radius, double degrees);
int stop();
int idle();
//line sensor
void lineSens_calib();
int lineSens_min();
double centerMass(char color);
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


void  reset_odo(odotype * p)
{
	p->right_pos = p->left_pos = 0.0;
   	p->right_enc_old = p->right_enc;
   	p->left_enc_old = p->left_enc;

  	p->x_pos = 0.0;
  	p->y_pos = 0.0;
  	p->theta_pos = 0.0;
	p->theta = 0.0;
  	p->old_theta = 0.0;
  	
  	p->theta_ref = 0.0;
}

void update_odo(odotype *p)
{
  	int delta;
  	double U,Ur,Ul;

  	delta = p->right_enc - p->right_enc_old;
  	if (delta > 0x8000) 
    	delta -= 0x10000;
  	else if (delta < -0x8000) 
    	delta += 0x10000;

  	p->right_enc_old = p->right_enc;
  	p->right_pos += delta * p->cr;
  	double right_pos_d = delta* p->cr;

  	delta = p->left_enc - p->left_enc_old;
  	if (delta > 0x8000)
    	delta -= 0x10000;
  	else if (delta < -0x8000)
    	delta += 0x10000;

  	p->left_enc_old = p->left_enc;
  	p->left_pos += delta * p->cl;
  	double left_pos_d = delta * p->cl;

  	Ul = left_pos_d; 
  	Ur = right_pos_d; 
  	U = (Ur+Ul)/2;
  	
	p->theta += (Ur-Ul) / p->w;

  	p->theta_pos = p->theta_pos + (Ur-Ul) / p->w;

  
  	//printf("\n		Theta Before: %f",p->theta_pos);
  
  	if (p->theta_pos > M_PI)
  		p->theta_pos -= 2*M_PI;
  	else if(p->theta_pos <= -M_PI)
  		p->theta_pos += 2*M_PI;
  
  	//printf("\n		Theta after: %f",p->theta_pos);
  	
  	
  	p->x_pos = p->x_pos + U*cos(p->theta_pos);
  	p->y_pos = p->y_pos + U*sin(p->theta_pos);
}


void update_motcon(motiontype *p){ 

	double theta;
	double v_max_ang; 
	double d;
	double v_max;
	double delta_v;
	double k = 0.5;
	int ls;
	double k_l = 0.2;
	

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
			
			case mot_line:
       			p->startpos=(p->left_pos+p->right_pos)/2;
       			p->curcmd=mot_line;
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

	/*************************** MOT MOVE ************************************/		 
     	case mot_move:
       		if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist)
       		{
          		p->finished=1;
	        	p->motorspeed_l=0;
          		p->motorspeed_r=0;
       		}	  
       		else 
       		{
       			d = fabs(p->dist) - (fabs(p->right_pos+p->left_pos)/2 - fabs(p->startpos));
       			v_max = sqrt(2*0.5*d);
       			
       			delta_v = k*(odo.theta_ref-odo.theta);
		
				//printf("\nSpeed: %f",Speed);
				//printf("\nMAximum Speed: %f",v_max);
				//printf("\nremaing distance: %f",d);	
			
				if(Speed <= v_max)
				{
					if (Speed < p->speedcmd)
						Speed += 0.005; 
				}
				else
				{ 
					Speed -= 0.005; 
				}
				//printf("\nref: %f  odo: %f  deltaV: %f",odo.theta_ref,odo.theta,delta_v);

				p->motorspeed_l = Speed - delta_v;
				p->motorspeed_r = Speed + delta_v;				
				
		   	}    
     		break;
	 /*************************** MOT LINE ************************************/
	 case mot_line:
	 		ls = lineSens_min();
       		if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist || ls_calib[ls] > 0.2) 	//if not more black STOP
       		{
          		p->finished=1;
	        	p->motorspeed_l=0;
          		p->motorspeed_r=0;
                  /*
                  printf("\nline sensor:\n");
                  for (int i = 0; i < 8; i++)
                  {
                      printf("%f\t",ls_calib[i]);
                  }
                  printf("\n");
                  */
                  
       		}	  
       		else 
       		{
       			d = fabs(p->dist) - (fabs(p->right_pos+p->left_pos)/2 - fabs(p->startpos));
       			v_max = sqrt(2*0.5*d);			

					/*
				if (ls == 3 || ls == 4)
					delta_v = 0;
				else
					delta_v = 0.2*(3.5-ls);
					*/
                delta_v = 0;
				switch(mot.direction)
				{
					case 'm':
						delta_v = k_l*(3.5-centerMass('b'));
						break;
					case 'l':
						delta_v =k_l*(1.5-centerMass('b'));
						break;
					case 'r':
						delta_v = k_l*(5.5-centerMass('b'));
						break;
				}
                //printf("\ncenter of mass: %f",centerMass('b'));
                //printf("\ndelta v: %f",delta_v);

				//printf("\nSpeed: %f",Speed);
				//printf("\nMAximum Speed: %f",v_max);
				//printf("\nremaing distance: %f",d);	
			
				if(Speed <= v_max)
				{
					if (Speed < p->speedcmd)
						Speed += 0.005; 
				//	printf("\nspeed+");
				}
				else
				{ 
					if (Speed > 0.005)
					Speed -= 0.005; 
				//	printf("\nspeed-");
				}
			//printf("Speed: %f",Speed);

				p->motorspeed_l = Speed + delta_v;
				p->motorspeed_r = Speed - delta_v;				
				
		   	}    
     		break;
     
     /*************************** MOT TURN ************************************/
     
     	case mot_turn:
     		if (p->angle>0)
     		{	// if (current angel < requested)
	  			if ( (fabs(odo.theta_pos - odo.old_theta)*p->w) < (p->angle*p->w) )
      			{
			  		//printf("\nRIGHT requested angle= %f",p->angle);
			  		//printf("\nRIGHT current angle= %f",fabs(odo.theta_pos - odo.old_theta));
			  		
			  		// Theta is distant left - theta = requested angle - current angle
			  		theta = (p->angle*p->w) - (fabs(odo.theta_pos - odo.old_theta)*p->w);
			  	
			  	
			  		//printf("\nRIGHT Theta left= %f",theta);
					v_max_ang = sqrt(2*0.5*theta);
				
					if(Speed < v_max_ang)
					{
						if (Speed < p->speedcmd)
						 	Speed += 0.005; 
					}		
					else
					{
						Speed -= 0.005; 
					}
				
					//printf("\nSpeed mot.turn= %f",Speed);
					p->motorspeed_r=Speed;
					p->motorspeed_l=-Speed;
				}		
				else 
				{	     
				    p->motorspeed_r=0;
				    p->motorspeed_l=0;
				    p->finished=1;
	  			}
			}
	     /*************************** Part 2 ************************************/
			else 
			{
				if (fabs(odo.theta_pos - odo.old_theta)*p->w < fabs(p->angle*p->w))
	  			{
	  				
				  	//printf("\nLEFT requested angle= %f", fabs(p->angle));
				  	//printf("\nLEFT current angle= %f",fabs(odo.theta_pos - odo.old_theta));
				  	//printf("\nLEFT Speed: %f",Speed);
				  	
				  	// Theta is distant left - theta = requested angle - current angle
				  	theta = fabs(p->angle*p->w) - (fabs(odo.theta_pos - odo.old_theta)*p->w);
					
					//printf("\nLEFT Angle left= %f",theta);
					
					v_max_ang = sqrt(2*0.5*theta);
						
					if(Speed < v_max_ang)
					{
						if (Speed < p->speedcmd)
							Speed += 0.005; 
					}		
					else
					{
						Speed -= 0.005; 
					}
					
					p->motorspeed_l=p->speedcmd;
				 	p->motorspeed_r=-(p->speedcmd);
				}
			  	else 
				{
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
   
	if (time==0)
   	{
		odo.theta_ref = odo.theta;	// Den teoretiske vinkel
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
		odo.old_theta = odo.theta_pos; 
     	mot.cmd=mot_turn;
     	mot.speedcmd=speed;
     	mot.angle=angle;
     	return 0;
   	}
 	else
   		return mot.finished;
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

void lineSens_calib()
{
	for (int i = 0; i < 8; i++)
	{
		ls_calib[i] = (linesensor->data[i]-lineBlack)/(lineWhite-lineBlack);
	}
}

int lineSens_min()
{
	lineSens_calib();
	int lowest = 0;

	for(int i = 1; i < 8;i++)
	{
		//printf("calib lowest: %f calib current: %f",ls_calib[lowest],i)
		//printf("\ncalib lowest: %d calib current: %d",lowest,i);
		if (ls_calib[lowest] > ls_calib[i])
		{
			lowest = i;
		}
		
	}
	return lowest;
}

/**
* \param direction 'm' middel / 'r' right / 'l' left
**/
int fl(double dist, double speed,int time,char direction)
{
   
	if (time==0)
   	{     	
		mot.direction=direction;   
		mot.cmd=mot_line;
     	mot.speedcmd=speed;
     	mot.dist=dist;  
     	return 0;
   	}
   	else
   	{
     	return mot.finished;
   	}
}

 
/**
* \param color 'w' if white and 'b' if black
**/
double centerMass(char color)
{
	lineSens_calib();
	double x_cu = 0;
	double x_cd = 0.00000000000001;

	switch(color)
	{
		case 'w':
			for (int i = 0; i < 8; i++)
			{
				x_cu += i*ls_calib[i];
				x_cd +=ls_calib[i];
			}
			
			break;
		case 'b':
			for (int i = 0; i < 8; i++)
			{
				x_cu += i*(1-ls_calib[i]);
				x_cd +=1-ls_calib[i];
			}

			break;
	}

	return x_cu/x_cd;

}

int crossingblackline() 
{
    lineSens_calib();
    double sum = 0;
    for (int i = 0; i < 8; i++)
    {
        sum += ls_calib[i];
    }
    //if(sum < 0.2*7+)
    return sum <= lineBlack*9 ? 1 : 0;
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
