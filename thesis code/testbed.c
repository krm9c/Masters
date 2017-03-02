

#include "testbedheader.h"




float truck_accel_x[6],truck_accel_y[6],truck_accel_z[6],truck_pot[6],
	  truck_steer[6],truck_speed[6]; /* pointers to respective sensor values */

pthread_mutex_t adc_mutex = PTHREAD_MUTEX_INITIALIZER;//assigning mutex for ADC access function
 
static long int current_state = 0;/*global variable defining current state */

int current_accelerometer_x,current_accelerometer_y,current_accelerometer_z,
           current_pot,current_speed,current_steering;//global variables defining current updates in range of 0 to 5 of respective sensors

void* truck_accelerometer_x(void *filehandle1)//truck accelration in x direction update thread

 {	
	RTIME period_of_update1;
        RT_TASK *thread_truck_accel_x;
	thread_truck_accel_x = rt_task_init(nam2num("truckaccel_x"), 0, 1024, 0);
	period_of_update1 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_truck_accel_x, rt_get_time() + period_of_update1,
	period_of_update1);
	int x_adc;
	int i=0;
	int fd1 = (int)filehandle1;
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 	
        while(1) {
		  current_accelerometer_x = i;
		  pthread_mutex_lock(&adc_mutex); // initiate mutex lock
		  x_adc =aim104_multi_io_ADC(fd1,3,1);
			//x_adc is ADC output across channel3(xout of accelerometer)
		  pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
	          truck_accel_x[i] = ((x_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g
		  i=i+1;
		  if (i>5) i=0;		  
		  rt_task_wait_period(); /*1ms*/
                }
       	       
	         
        }




void* truck_accelerometer_y(void *filehandle2)//truck accelration in y direction update thread

 {	
	RTIME period_of_update2;
	RT_TASK *thread_truck_accel_y;
	thread_truck_accel_y = rt_task_init(nam2num("truckaccel_y"), 0, 1024, 0);
	period_of_update2 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_truck_accel_y, rt_get_time() + period_of_update2,
	period_of_update2);
	int y_adc;
	static int i=0;
	int fd1 = (int)filehandle2;
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
	
      while(1) {
		  current_accelerometer_y = i;
		  pthread_mutex_lock(&adc_mutex);// initiate mutex lock
		  y_adc =aim104_multi_io_ADC(fd1,4,1);
			//y_adc is ADC output across channel4(yout of accelerometer)
		  pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
	          truck_accel_y[i] = ((y_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g		  
		  i=i+1;
		  if (i>5) i=0;		  
                  rt_task_wait_period(); /*1ms*/
                }
 }



	
void* truck_accelerometer_z(void *filehandle3)//truck accelration in z direction update thread

 {	
	RTIME period_of_update3;
	RT_TASK *thread_truck_accel_z;
	thread_truck_accel_z = rt_task_init(nam2num("truckaccel_z"), 0, 1024, 0);
	period_of_update3 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_truck_accel_z, rt_get_time() + period_of_update3,
	period_of_update3);
        float z_adc;
	static int i=0;
	int fd1 = (int)filehandle3;
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
	
        while(1) {
		  current_accelerometer_z = i;
		  pthread_mutex_lock(&adc_mutex);// initiate mutex lock
		  z_adc =aim104_multi_io_ADC(fd1,5,1);
			//z_adc is ADC output across channel5(zout of accelerometer)
		  pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
	          truck_accel_z[i] = ((z_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g
		  i=i+1;
		  if (i>5) i=0;
		  rt_task_wait_period(); /*1ms*/
                }
       	        
        }

void* truck_potentiometer(void *filehandle4)// trailer cab angle potentiometer update thread
 {	
	RTIME period_of_update4;
	RT_TASK *thread_truck_pot; 
	thread_truck_pot = rt_task_init(nam2num("truckpot"), 0, 1024, 0);
	period_of_update4 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_truck_pot, rt_get_time() + period_of_update4,
	period_of_update4);
        int truck_pot_adc;
	float pi=(22/7);
	int i=0;
	int fd1 = (int)filehandle4;
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
	//pthread_mutex_lock(&adc_mutex);// initiate mutex lock
        while(1) {
		    current_pot= i;
		    pthread_mutex_lock(&adc_mutex);// initiate mutex lock
		    truck_pot_adc =aim104_multi_io_ADC(fd1,0,1);
			 //truck_pot_adc is ADC output across channel0(zout of accelerometer)
	  	  pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
	      truck_pot[i] = ((340*truck_pot_adc/4095)-170)*(pi/180);
			//truck cab-trailer angle from -170 deg to 170 deg in radians
		     i=i+1;
		    if (i>5) i=0;
		      rt_task_wait_period(); /*1ms*/
        }
       	       //pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
      }



void* truck_steering (void *filehandle5)// trailer cab angle potentiometer update thread

 {	
	RTIME period_of_update5;
	RT_TASK *thread_truck_steer;
	thread_truck_steer = rt_task_init(nam2num("trucksteer"), 0, 1024, 0);
	period_of_update5 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_truck_steer, rt_get_time() + period_of_update5,
	period_of_update5);
        int truck_steer_adc;
	float pi=(22/7);
	int i=0;
	int fd1 = (int)filehandle5;
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
	
        while(1) {
		  current_steering = i;
		  pthread_mutex_lock(&adc_mutex);// initiate mutex lock
		  truck_steer_adc =aim104_multi_io_ADC(fd1,6,1);
			//truck_steer_adc is ADC output across channel6
		  pthread_mutex_unlock(&adc_mutex);// initiate mutex unlock
	          truck_steer[i] = ((15*truck_steer_adc/4095)-30)*(pi/180);
			//steering angle from -15 deg to +15 deg in radians
		  i=i+1;
		  if (i>5) i=0;
		  
		  rt_task_wait_period() ;/*1ms*/
                }
	}
       	       
	

void current_state_K(void )

 {	
	RTIME period_of_update6;
	RT_TASK *thread_current_state_k;
	thread_current_state_k = rt_task_init(nam2num("currstate"), 0, 1024, 0);
	period_of_update6 = nano2count(TICKPERIOD);
	rt_task_make_periodic(thread_current_state_k, rt_get_time() + period_of_update6,
	period_of_update6);
	static int i,j,k;
		 
        rt_make_hard_real_time();//rtai lxrt module function call to make hard real time
	while (1)
	{ 
        for (i=0;i<=1000;i++)
	       for (j=0;j<=1000;j++)
		 for (k=0;k<=1000;k++)
			  {
			 	current_state=current_state + 1;
			 	}   
	if (current_state = 1000000000) current_state = 0;
	rt_task_wait_period(); /*1ms*/       
					}      
 
 }
	       
 float truck_potentiometer_update (int n)// returns nth previous update of trailer-cab angle
{	
	int update_number;
	RT_TASK *truck_pot_update;
	truck_pot_update = rt_task_init(nam2num("truckpotupdate"), 0, 1024, 0);
   	rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
   	if ((current_pot - n)<0) 
      		update_number = 6+current_pot-n;
   	else
		update_number = current_pot-n;
	return truck_pot[update_number];
}

float truck_steering_update (int n)//return nth previous update of truck steering angle
{	
	int update_number;
	RT_TASK *truck_steer_update;
	truck_steer_update = rt_task_init(nam2num("trucksteerupdate"), 0, 1024, 0);
   	rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
   	if ((current_steering - n)<0) 
      		update_number = 6+current_steering-n;
   	else
		update_number = current_steering-n;
	return truck_steer[update_number];
}



float truck_accelerometer_x_update (int n)//returns nth previous update of acceleration in x direction
{	
	int update_number;
	RT_TASK *truck_accel_x_update;
	truck_accel_x_update = rt_task_init(nam2num("truckaccelxupdate"), 0, 1024, 0);
   	rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
   	if ((current_accelerometer_x  - n)<0) 
      		update_number = 6 + current_accelerometer_x -n;
   	else
		update_number = current_accelerometer_x -n ;
	return truck_accel_x[update_number];
}

float truck_accelerometer_y_update (int n)//returns nth previous update of acceleration in y direction
{	
	int update_number;
	RT_TASK *truck_accel_y_update;
	truck_accel_y_update = rt_task_init(nam2num("truckaccelyupdate"), 0, 1024, 0);
   	rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
   	if ((current_accelerometer_y  - n)<0) 
      		update_number = 6+current_accelerometer_y -n;
   	else
		update_number = current_accelerometer_y -n;
	return truck_accel_y[update_number];
}

float truck_accelerometer_z_update (int n)// returns nth previous update of acceleration in z direction
{
	int update_number;
	RT_TASK *truck_accel_z_update;
	truck_accel_z_update = rt_task_init(nam2num("truckaccelzupdate"), 0, 1024, 0);
   	rt_make_hard_real_time();//rtai lxrt module function call to make hard real time 
   	if ((current_accelerometer_z  - n)<0) 
      		update_number = 6+current_accelerometer_z -n;
   	else
		update_number = current_accelerometer_z -n;
	return truck_accel_z[update_number];

}

//******** 'motor' record initialization (assigns motor to card half) ********

// a call would be AssignMotor(&motorx, card, whichhalf);

void AssignMotor(Motor *motorx, int card, int whichhalf)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl0"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  switch (whichhalf) {

   case 0:

    {

      motorx->LM629PortLocation = card + I27LM629_0;

      motorx->AuxMotorDisableBit = Motor0Disable;

      motorx->AuxSenseMask = Motor0Sense;

      motorx->AuxOvertempMask = Driver0Overtemp;

      motorx->AuxIndexMask = Motor0Index;

      break;

    } //whichhalf = 0

    case 1:  {

      motorx->LM629PortLocation = card + I27LM629_1;

      motorx->AuxMotorDisableBit = Motor1Disable;

      motorx->AuxSenseMask = Motor1Sense;

      motorx->AuxOvertempMask = Driver1Overtemp;

      motorx->AuxIndexMask = Motor1Index;

    } //whichhalf = 1

  } //switch

  motorx->AuxOutPortLocation = card + I27PPIA;

  motorx->AuxInPortLocation = card + I27PPIB;

} // AssignMotor

//******** low level routines for auxillary IO ports ********

void InitPPI( int card)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl1"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  outp(card + I27PPICont, OutAInBC);  // set to normal directions

  outp(card + I27PPIA, DisIntDriveBit|LightOffBit|Motor0Disable|Motor1Disable);

}



void EnableHBridge(Motor *motorx)       //applies only to 7I25

{

  RT_TASK *task;
  task = rt_task_init(nam2num("rtl2"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

char portsave;

  portsave = inp(motorx->AuxOutPortLocation);

  portsave = portsave & (!(motorx->AuxMotorDisableBit));

  outp(motorx->AuxOutPortLocation, portsave);

}



void DisableHBridge(Motor *motorx)    //applies only to 7I25

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl3"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

char portsave;

  portsave = inp(motorx->AuxOutPortLocation);

  portsave = portsave | motorx->AuxMotorDisableBit;

  outp(motorx->AuxOutPortLocation, portsave);

}



void LightOn(int card)

{

char portsave;

  portsave = inp(card +I27PPIA);

  portsave = portsave & !LightOffBit;

  outp(card + I27PPIA, portsave);

}



void LightOff(int card)

{

  char portsave;

  portsave = inp(card + I27PPIA);

  portsave = portsave | LightOffBit;

  outp(card + I27PPIA, portsave);

}



void EnableInterruptDriver(int card)

{

char portsave;

  portsave = inp(card + I27PPIA);

  portsave = portsave & (! DisIntDriveBit);

  outp(card + I27PPIA, portsave);

}



void DisableInterruptDriver(int card)

{

char portsave;

  portsave = inp(card + I27PPIA);

  portsave = portsave | DisIntDriveBit;

  outp(card + I27PPIA, portsave);

}



int DriverOverTemp(Motor *motorx)

{

  return ((inp(motorx->AuxInPortLocation) & motorx->AuxOvertempMask) == 0);

}



int AuxSense(Motor *motorx)

{

  return ((inp(motorx->AuxInPortLocation) & motorx->AuxSenseMask) == 1);

}



int AuxIndexSense(Motor *motorx)

{

  return ((inp(motorx->AuxInPortLocation) & motorx->AuxIndexMask) == 0);

}




//procedures to read velocities and position


long ReadIndexPosition(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl4"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,RdipCmd);

  return ReadLongint(motorx);

}



long ReadDesiredPosition(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl5"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,RDDPCmd);

  return ReadLongint(motorx);

}



long ReadRealPosition(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl6"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time
  

  WriteCommand(motorx,RDRPCmd);

  return ReadLongint(motorx);

}



long ReadDesiredVelocity(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl7"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,RDDVcmd);

  return ReadLongint(motorx);

}



long ReadRealVelocity(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl8"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time
  
  WriteCommand(motorx,RDRVcmd);

  return ReadLongint(motorx);

}



//motor control read write commands

void WriteCommand(Motor *motorx, char cmd)

{ 
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl19"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time
  

  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  outp(motorx->LM629PortLocation, cmd);

}



void WriteDataWord(Motor *motorx , unsigned int data)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl20"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time
  

  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  outp((motorx->LM629PortLocation + LM629DataRegOffset), (data >> 8));

  outp((motorx->LM629PortLocation + LM629DataRegOffset), data);

}



unsigned int ReadDataWord(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl21"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  unsigned int high, low;

  
  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  high = (inp(motorx->LM629PortLocation + LM629DataRegOffset) << 8);

  low  = (inp(motorx->LM629PortLocation + LM629DataRegOffset)) & 0x00ff;

  return (high | low);

}



void WriteLongint(Motor *motorx, unsigned long data)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl22"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteDataWord(motorx, (unsigned int)((data & 0xffff0000)  >> 16) ); /*high int*/

  WriteDataWord(motorx, (unsigned int)(data & 0x0000ffff )); /*low int*/

}



unsigned long ReadLongint(Motor *motorx)

{

  RT_TASK *task;
  task = rt_task_init(nam2num("rtl23"),1,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

unsigned long high;

unsigned long low;

  
  high = ((unsigned long)ReadDataWord(motorx)) << 16;

  low  = ((unsigned long)ReadDataWord(motorx)) & 0x0000ffff;

return  (high | low);

}

//procedure to turn on motor

void TurnOnMotor(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl24"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,TurnOnMotorCmd);

  WriteCommand(motorx,SttCmd);

}



/*these are emergency measures...*/

void TurnOffMotor(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl25"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,TurnOffMotorCmd);

  WriteCommand(motorx,SttCmd);

}



void StopSmoothly(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl26"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,StopSmoothlyCmd);

  WriteCommand(motorx,SttCmd);

}



void StopAbruptly(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl27"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,StopAbruptlyCmd);

  WriteCommand(motorx,SttCmd);

}

//yet to be commented

int LM629Busy(Motor *motorx)

{

return ((inp(motorx->LM629PortLocation) & LM629BusyMask) != 0 );

}



int TrajectoryComplete(Motor *motorx)

{

return ((ReadLM629Status(motorx)&(TrajectoryComMask|CommandErrorMask|ExcessivePosMask)) != 0);

}



void WaitForTrajectoryComplete(Motor *motorx)

{

  while (!TrajectoryComplete(motorx)) {;} /*wait*/

}

//yet to be commented

int ReadSignals(Motor *motorx) /*tca changed - removed "signals"*/

{

  WriteCommand(motorx,RdsigsCmd);

  return ReadDataWord(motorx);

}

long ReadIntegraterSum(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl28"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,RDSUMcmd);

  return ReadLongint(motorx);

}

//******** Lowest level LM629 routines ********

char ReadLM629Status(Motor *motorx)

{

  int LM629BugDelay;



  LM629BugDelay = 0;

  while (LM629BugDelay < 1000 ) ++LM629BugDelay;

  // This nonsense is because of a NS LM629 chip bug -- If you poll the status

  //  register too fast, the LM629 crashes. This seems to be a feature on newer

  //  (DC >89 or so) chips

  return inp(motorx->LM629PortLocation);

}

/******** Low level LM629 Verbs *********/

int ResetLM629(int card, int whichhalf)

{

int i;

int theport;

  switch (whichhalf) {

    case  0 : 	theport = I27LM629_0 + card;

		break;

    case  1 : 	theport = I27LM629_1 + card;

  }/* case*/

  outp(theport, RESETcmd);

for(i=0; i<30000; i++);/*wait around*/ 

/*   delay(5);                      /wait for 5 milliseconds*/

  if ((inp(theport) & ResetDataMask) != ResetData ) return 0;

  else return 1;

}


//yet to comment

/*these reset with the home and index positions to the current position*/

void DefineHome(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl29"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,DfhCmd);

  motorx->LastPosition = 0;

}



void SetIndexPosition(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl30"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,SipCmd);

}



/*this command starts a pre-loaded trajectory*/

void StartTrajectory(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl31"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  WriteCommand(motorx,SttCmd);

}



/*this command updates the filter coefficients from their (pre-loaded) buffers*/

void UpdateFilter(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl32"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,UdfCmd);

}



void Port8Command(Motor *motorx)

{

  WriteCommand(motorx,Port8Cmd);

}



void LoadPosErrForInt(Motor *motorx, int maxerr)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl33"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LPEICmd);

  WriteDataWord(motorx,maxerr);

}



void LoadPosErrForStop(Motor *motorx, int maxerr)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl34"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LPESCmd);

  WriteDataWord(motorx,maxerr);

}



void SetBreakpointAbsolute(Motor *motorx, long position)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl35"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,SBpACmd);

  WriteLongint(motorx,position);

}



void SetBreakpointRelative(Motor *motorx, long position)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl36"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,SBpRCmd);

  WriteLongint(motorx,position);

}



void MaskInterrupts(Motor *motorx, unsigned char mask)

{

  WriteCommand(motorx,MskICmd);

  WriteDataWord(motorx,(unsigned int)mask);

}



void ResetInterrupts(Motor *motorx, unsigned char mask)

{

  WriteCommand(motorx,RstICmd);

  WriteDataWord(motorx,(unsigned int)mask);

}

/********  procedures for loading filter and trajectory data to LM629 *********/

void LoadFilterParameters(Motor *motorx)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl37"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


unsigned int filtcontword;

  WriteCommand(motorx,LFilCmd);

  filtcontword = motorx->SamplingInterval;

  filtcontword = filtcontword | LoadAllFil;

  WriteDataWord(motorx,filtcontword);

  WriteDataWord(motorx,motorx->Filter.KP);

  WriteDataWord(motorx,motorx->Filter.KI);

  WriteDataWord(motorx,motorx->Filter.KD);

  WriteDataWord(motorx,motorx->Filter.IL);

}



void LoadKP(Motor *motorx, unsigned int kp)

{ 
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl38"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKPBit);

  WriteDataWord(motorx,kp);

}



void LoadKI(Motor *motorx, unsigned int ki)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl39"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKIBit);

  WriteDataWord(motorx,ki);

}



void LoadKD(Motor *motorx, unsigned int kd)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl40"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKDBit);

  WriteDataWord(motorx,kd);

}



void LoadIL(Motor *motorx, unsigned int il)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl41"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadILBit);

  WriteDataWord(motorx,il);

}



void LoadTrajectory(Motor *motorx, Trajectory *traj)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl42"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadAllTraj);

  WriteLongint(motorx,traj->Acceleration);

  WriteLongint(motorx,traj->Velocity);

  WriteLongint(motorx,traj->Position);

}



void LoadAcceleration(Motor *motorx, unsigned long acceleration)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl43"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadAccBit);

  WriteLongint(motorx,acceleration);

}



void LoadVelocity(Motor *motorx, unsigned long velocity)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl44"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadVelBit);

  WriteLongint(motorx,velocity);

}



void LoadPosition(Motor *motorx, long position)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl45"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadPosBit);

  WriteLongint(motorx,position);

}

/******** mid level commands *********/

int Reset6I27(int card)

{

int resetok;



  InitPPI(card);                       /*set up 8255 port*/

  LightOn(card);                       /*turn light on for debugging*/

  resetok = ResetLM629(card,0);        /*reset first LM629*/

  resetok = ResetLM629(card,1) && resetok;   /*reset second LM629*/

  LightOff(card);                  /*turn off lights before leaving...*/

  return resetok;

}



void SetSamplingInterval(Motor *motorx, unsigned int sinterval)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl46"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  sinterval = (sinterval << 8 );

  motorx->SamplingInterval = sinterval;

}



void SetMotionMode(Motor *motorx, int mmode)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl47"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  motorx->MotionMode = mmode;

}



void SetEncoderlines(Motor *motorx, int lines)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl48"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time

  motorx->EncoderLines = lines;

}



void SetClockFrequency(Motor *motorx, long frequency)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl49"),2,1024,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


  motorx->ClockFrequency = frequency;

}



void LoadVelocityInRPM(Motor *motorx, float velocity)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl50"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


float samplePeriod;

long countsPerSample;

  samplePeriod = (motorx->SamplingInterval +1)/(motorx->ClockFrequency / 2048);

  countsPerSample = (long)((samplePeriod * motorx->EncoderLines * (4/60) * velocity * 65536));

/*X 4 because of four transitions per line on encoder, /60 for rpM, X 65536 to make 16 bit

 number with 16 bit fraction*/

  LoadVelocity(motorx,countsPerSample);

}



char PolledMove(Motor *motorx, long position)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl51"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


char status;

  LoadPosition(motorx,position);

  StartTrajectory(motorx);

  WaitForTrajectoryComplete(motorx);

  motorx->LastPosition = position;

  return status;

}



/*this is a non-synchronized 2 axis move (for indexing) - the path is not a straight line

this procedure expects that the velocity and acceleration values have already been loaded

for both motors*/

char PolledXYIndex(Motor *motorx, Motor *motory, long newx, long newy)

{
  RT_TASK *task;
  task = rt_task_init(nam2num("rtl52"),2,4096,0);
  rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time


char status;

  LoadPosition(motorx,newx);

  LoadPosition(motory,newy);

  StartTrajectory(motorx);

  StartTrajectory(motory);

  while (!(TrajectoryComplete(motorx) && TrajectoryComplete(motory))) {;} /*wait;*/

  motorx->LastPosition = newx;

  motory->LastPosition = newy;

  return ReadLM629Status(motorx) | ReadLM629Status(motory);

}

void init_threads(void* adcFD)

{ 
  	

       	RT_TASK *task;
  	task = rt_task_init(nam2num("initialize"),0,4096,0);
  	rt_make_hard_real_time();//rtai lxrt module function calls to make hard real time
	pthread_t th0,th1,th2,th3,th4,th5; //7 threads declared

//create/run all threads
	
  	pthread_create(&th0, NULL, truck_potentiometer,(void *)adcFD);
  	pthread_create(&th1, NULL, truck_steering, (void *)adcFD);
  	pthread_create(&th2, NULL, truck_accelerometer_x, (void *)adcFD);
  	pthread_create(&th3, NULL, truck_accelerometer_y, (void *)adcFD);
  	pthread_create(&th4, NULL, truck_accelerometer_z, (void *)adcFD);
  	

// join all threads
        pthread_join(th0, 0);
        pthread_join(th1, 0);
        pthread_join(th2, 0);
	pthread_join(th3, 0);
	pthread_join(th4, 0);
	                
	
}

