/*
 * An example SMR program.
 */

#include "include_all.h"
#define ROBOTPORT	8000 //24902

int main()
{    
    // Connection.
    
    // **************************************************
    //  Establish connection to robot sensors and actuators
    // **************************************************
    int running,arg,time=0;
    if (rhdConnect('w',"localhost",ROBOTPORT)!='w')
    {
        printf("Can't connlect to rhd \n");
        exit(EXIT_FAILURE); 
    }    
        
    printf("connected to robot \n");    

    if ((inputtable=getSymbolTable('r'))== NULL)
    {
        printf("Can't connect to rhd \n"); 
        exit(EXIT_FAILURE); 
    } 
        
    if ((outputtable=getSymbolTable('w'))== NULL)
    { 
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
    // **************************************************

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

    if (camsrv.config) 
    {
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
    if (lmssrv.config) 
    {
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
            len=sprintf(buf,"scanpush cmd='zoneobst'\n");
            send(lmssrv.sockfd,buf,len,0);
        }
    }   
        
        
    /* Read sensors and zero our position.  */
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
    int statemachine=ms_obs1; 
    
    
    /**********************************************************************************
     *                                 MAIN LOOP START
     **********************************************************************************/
    while (running)
    {  
        if (lmssrv.config && lmssrv.status && lmssrv.connected)
        {
            while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
                xml_proca(xmllaser);
        }
        
        if (camsrv.config && camsrv.status && camsrv.connected)
        {
            while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
                xml_proc(xmldata);
        }
         
        rhdSync();
        odo.left_enc=lenc->data[0];           
        odo.right_enc=renc->data[0];      
        update_odo(&odo);   
             
        /****************************************\         
                      statemachine                   
        \******************* *********************/       

        switch (statemachine) {    
            case ms_obs1:
                if(run_obstacle_1() == 1) statemachine = ms_obs2; 
                //printf("Obs %d\n", statemachine+1); // the first  case is 0, but we call it obs_1, so -> +1
                break; 
            case ms_obs2:  
                if(run_obstacle_2() == 1) statemachine = ms_obs3;
                //printf("Obs %d\n", statemachine+1);               
                break;
            case ms_obs3:   
                if(run_obstacle_3() == 1) statemachine = ms_obs4;
                printf("Obs %d\n", statemachine+1);
                break;
            case ms_obs4:
                if(run_obstacle_4() == 1) statemachine = ms_obs5;
                printf("Obs %d\n", statemachine+1);
                break;
            case ms_obs5:
                if(run_obstacle_5() == 1) statemachine = ms_obs6;
                printf("Obs %d\n", statemachine+1);
                break;
            case ms_obs6:
                if(run_obstacle_6() == 1) statemachine = ms_end;
                printf("Obs %d\n", statemachine+1);
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
        speedl->data[0]=100*mot.motorspeed_l;
        speedl->updated=1;
        speedr->data[0]=100*mot.motorspeed_r;
        speedr->updated=1;

        // 100 Hz clock, which means this is true once every 1 second.
        if (time  % 100 == 0)
        //    printf(" laser %f \n",laserpar[3]);
        time++;
        /* stop if keyboard is activated
        *
        */
        ioctl(0, FIONREAD, &arg);
        if (arg!=0)  running=0;
    }
    /**********************************************************************************
     *                                 MAIN LOOP END
     **********************************************************************************/

    // Finished Mission    
    speedl->data[0]=0;
    speedl->updated=1;
    speedr->data[0]=0;
    speedr->updated=1;
    rhdSync();
    rhdDisconnect();
    exit(0);
}  