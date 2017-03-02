#include <stdio.h>
#include <libaim104.h> // library supporting ADC functions
#include <math.h>
#include <time.h>
#include <linux/module.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>
#include <rtai_lxrt.h>//module to develop real time functions in user space
#include <pthread.h>//pthread library
#include <stdlib.h>



/* these are the offsets from the card base address to the individual ports*/

#define I27PPIA       	     0x00 /* port A - outputs Hbridge disable bits, LED, and Int driver enable*/

#define I27PPIB              0x01 /* port B - inputs sense, overtemp and auxindex bits*/

#define I27PPIC              0x02 /* 8 bit user parallel port... undefined*/

#define I27PPICont           0x03 /* control port for PPI*/

#define I27LM629_0           0x04 /* first LM629 control port*/

#define I27LM629_1           0x08 /* second LM629 control port*/

#define I27LM629_01          0x0C /* writes here write to both LM629's (don't read)*/

#define LM629DataRegOffset   0x01 /* LM629 data register accsessed with A0 high*/



/* PPI port data and masks*/

/* outputs on PPIA*/

#define Motor0Disable       0x01

#define Motor1Disable       0x02

#define LightOffBit         0x40

#define DisIntDriveBit      0x80



/* inputs on PPIB*/

#define Motor0Sense         0x01

#define Driver0Overtemp     0x02

#define Motor0Index         0x10

#define Motor1Sense         0x04

#define Driver1Overtemp     0x08

#define Motor1Index         0x20



/* And a useful PPI mode byte*/

#define OutAInBC            0x8B /* the normal PPIA mode... port A and B must be input and output*/

/*  respectivly, but port C can be 8 inputs, 8 outputs or 4 of each*/



/* basic LM629 command bytes (with  data sheet nmemonics)*/

#define ResetData          0x84    /* to look for a valid reset*/

#define ResetDataMask      0xBF



/* 'write' commands, no data*/

#define RESETcmd            0x00   /* reset the LM629*/

#define SttCmd              0x01   /* start motion*/

#define DfhCmd              0x02   /* define home*/

#define SipCmd              0x03   /* set index position*/

#define UdfCmd              0x04   /* update filter*/

#define Port8Cmd            0x05   /* select 8 bit output (pwm)*/

#define Port12Cmd           0x06   /* select 12 bit output (better not !)*/



/* 'write' commands, all followed by data */

#define LPESCmd             0x1A   /* stop on error*/

#define LPEICmd             0x1B   /* interrupt on error*/

#define MskICmd             0x1C   /* mask interrupts*/

#define RstICmd             0x1D   /* reset interrupts*/

#define LFilCmd             0x1E   /* load filter parameters*/

#define LTrjCmd             0x1F   /* load trajectory*/

#define SBpACmd             0x20   /* set breakpoint absolute*/

#define SBpRCmd             0x21   /* set breakpoint relative*/



/* 'ready' commands*/

#define RDDVcmd             0x07   /* read desired velocity*/

#define RDDPCmd             0x08   /* read desired position*/

#define RdipCmd             0x09   /* read index position*/

#define RDRPCmd             0x0A   /* read real position*/

#define RDRVcmd             0x0B   /* read real velocity*/

#define RdsigsCmd           0x0C   /* read signals register*/

#define RDSUMcmd            0x0D   /* read integration*/



/* status byte masks (same as low byte of signals register except bit 0)*/

#define MotorOffMask        0x80

#define BreakPointMask      0x40

#define ExcessivePosMask    0x20

#define WraparoundMask      0x10

#define IndexPulseMask      0x08

#define TrajectoryComMask   0x04

#define CommandErrorMask    0x02

#define LM629BusyMask       0x01



/* signals register masks (self explainatory)*/

#define HostInterruptMask   0x8000

#define AccelLoadedMask     0x4000

#define UDFExecutedMask     0x2000

#define ForwardDirMask      0x1000

#define VelocityModeMask    0x0800

#define OnTargetMask        0x0400

#define TurnOffOnEPEMask    0x0200

#define EightBitModeMask    0x0100

/* next six bits can use status register masks*/

#define AcqNextIdxMask      0x0001



/* load trajectory command (LTRJ) control bits*/

#define ForwardDirection    0x1000 // trajectory command for forward direction motion

#define VelocityMode        0x0800 //velocity mode selection

#define PositionMode        0x0000 // position mode selection

#define StopSmoothlyCmd     0x0400 // command to stop smoothly

#define StopAbruptlyCmd     0x0200 // command to stop abruptly

#define TurnOffMotorCmd     0x0100 // command to turn off motor

#define TurnOnMotorCmd      0x0000 // command to turn on motor

#define LoadAccBit          0x0020 // loading acceleration bit

#define AccRelBit           0x0010 // Acceleration bit	

#define LoadVelBit          0x0008 // loading velocity bit

#define VelRelBit           0x0004 // velocity bit

#define LoadPosBit          0x0002 // loading position bit

#define PosRelBit           0x0001 // relative position bit

#define LoadAllTraj         (LoadAccBit | LoadVelBit | LoadPosBit) //loading trajectories for 									acceleration, velocity and position



/* load filter command (LFIL) control bits*/

#define LoadKPBit           0x08 // command to load Kp of PID controller

#define LoadKIBit           0x04 // command to load Ki of PID controller

#define LoadKDBit           0x02 // command to load Kd of PID controller

#define LoadILBit           0x01 

#define LoadAllFil          (LoadKPBit | LoadKIBit | LoadKDBit | LoadILBit)

//motor control functions,structures and variable declarations ( self explainatory)

typedef struct {

  int  KP ;

  int  KI ;

  int  KD ;

  int  IL ; } FilterData;



typedef struct {

  long   Acceleration;

  long   Velocity;

  long   Position; } Trajectory;



typedef struct {

  int  LM629PortLocation;        /* where the (control and status) port is*/

  int  AuxOutPortLocation;       /* need to know for Hbridge disable bit*/

  char AuxMotorDisableBit; /* which bit is the disable bit for this motor*/

  int  AuxInPortLocation; /* need to know for reading sense, overtemp, and index*/

  char   AuxSenseMask;     /* where the sense bit is*/

  char   AuxOvertempMask;  /* where the Hbridge disable bit is*/

  char   AuxIndexMask;     /* where the index bit is*/

  long   LastPosition; /* previous position (for relative and multi-axis moves*/

  FilterData  Filter;           /* tuning parameters*/

  int  MotionMode ;             /* saved copy of motion mode*/

  int  SamplingInterval ;       /* saved copy of sampling interval*/

  long   ClockFrequency; /* LM629 clock frequency for acceleration and velocity scaling*/

  int  EncoderLines ; /* number of lines in motor encoder (resolution is 4X this number)*/

  } Motor ;



/********* 'motor' record initialization (assigns motor to card half) ********

 a call would be AssignMotor(&motorx, card, whichhalf);*/



void AssignMotor(Motor *motorx, int card, int whichhalf);

void InitPPI( int card); /* set port directions, disable motors and interrupts,

				 turn light off*/

void DisableInterrupts();

void EnableInterrupts();

void EnableHBridge(Motor *motorx); /*applies only to 7I25 - enables motor drive*/

void DisableHBridge(Motor *motorx);    /*applies only to 7I25*/

void LightOn(int card);

void LightOff(int card);

void EnableInterruptDriver(int card);

void DisableInterruptDriver(int card);

int DriverOverTemp(Motor *motorx);

int AuxSense(Motor *motorx);

int AuxIndexSense(Motor *motorx);

/********* Lowest level LM629 routines *********/

char ReadLM629Status(Motor *motorx);// read motor controller status

int LM629Busy(Motor *motorx);

int TrajectoryComplete(Motor *motorx);//routine to complete trajectory

void WaitForTrajectoryComplete(Motor *motorx);//routine to implement trajectory complete wait

void WriteCommand(Motor *motorx, char cmd);//routine to write commands used by other routines

void WriteDataWord(Motor *motorx , unsigned int data);// routine to write data word used by other 									routines	

unsigned int ReadDataWord(Motor *motorx);// routine to read data word used by other routines

void WriteLongint(Motor *motorx, unsigned long data);//routine to write long integer 

unsigned long ReadLongint(Motor *motorx);//routine to read long integer

/********* Miscellaneous LM629 read commands *********/

int ReadSignals(Motor *motorx); /* tca changed - removed "signals"*/

long ReadIndexPosition(Motor *motorx);// routine to read index position at any instant

long ReadDesiredPosition(Motor *motorx);//routine to read desired position at any instant

long ReadRealPosition(Motor *motorx);// routine to read real position at any instant

long ReadDesiredVelocity(Motor *motorx);//routine to read desired velocity at any instant

long ReadRealVelocity(Motor *motorx);// routine to read real velocity

long ReadIntegraterSum(Motor *motorx);// read integrator implementation result

/********* Low level LM629 Verbs *********/

int ResetLM629(int card, int whichhalf);//routine to reset either of LM629 motor controllers

void TurnOnMotor(Motor *motorx);//routine to turn on motor

/* these are emergency measures...*/

void TurnOffMotor(Motor *motorx);//routine to turn off motor

void StopSmoothly(Motor *motorx);//routine to stop motion smoothly

void StopAbruptly(Motor *motorx);//routine to stop motion abruptly

/* these reset with the home and index positions to the current position*/

void DefineHome(Motor *motorx);

void SetIndexPosition(Motor *motorx);//routine to set index position

/* this command starts a pre-loaded trajectory*/

void StartTrajectory(Motor *motorx);

/* this command updates the filter coefficients from their (pre-loaded) buffers*/

void UpdateFilter(Motor *motorx);

void Port8Command(Motor *motorx);

void LoadPosErrForInt(Motor *motorx, int maxerr);//routine to load position error for integration

void LoadPosErrForStop(Motor *motorx, int maxerr);//routine to load position error for stopping 							motion

void SetBreakpointAbsolute(Motor *motorx, long position);//routine to set absolute position 									breakpoint

void SetBreakpointRelative(Motor *motorx, long position);//routine to set absolute position 									breakpoint

void MaskInterrupts(Motor *motorx, unsigned char mask);// routine to mask interrupts

void ResetInterrupts(Motor *motorx, unsigned char mask);//routine to reset interrupts

/*********  procedures for loading filter and trajectory data to LM629 *********/

void LoadFilterParameters(Motor *motorx);//loading filter parameters 

//loading kp,ki,kd routines

void LoadKP(Motor *motorx, unsigned int kp);

void LoadKI(Motor *motorx, unsigned int ki);

void LoadKD(Motor *motorx, unsigned int kd);

//self explainatory

void LoadIL(Motor *motorx, unsigned int il);

void LoadTrajectory(Motor *motorx, Trajectory *traj);

void LoadAcceleration(Motor *motorx, unsigned long acceleration);

void LoadVelocity(Motor *motorx, unsigned long velocity);

void LoadPosition(Motor *motorx, long position);

/********* mid level commands (self explainatory) *********/

int Reset4I27(int card);

void SetSamplingInterval(Motor *motorx, unsigned int sinterval);

void SetMotionMode(Motor *motorx, int mmode);

void SetEncoderlines(Motor *motorx, int lines);

void SetClockFrequency(Motor *motorx, long frequency);

void LoadVelocityInRPM(Motor *motorx, float velocity);

char PolledMove(Motor *motorx, long position);

/* this is a non-synchronized 2 axis move (for indexing) - the path is not a straight line

 this procedure expects that the velocity and acceleration values have already been loaded

 for both motors */

char PolledXYIndex(Motor *motorx, Motor *motory, long newx, long newy);

/* this is a synchronized 2 axis move (for profiling) - the path is a strait line */

char PolledXYMove(Motor *motorx, Motor *motory,

long newx, long newy, float velocity, float acceleration);

//real time thread function,variable declarations


extern float     *truck_accel_x,*truck_accel_y,*truck_accel_z,*truck_pot,
	  *truck_steer,*truck_speed; // pointers to respective sensor values

static RTIME period_of_update = nano2count(1000*1000);//update period in timer counts of cpu

static RT_TASK thread_truck_pot,thread_truck_accel_x,thread_truck_accel_y,thread_truck_accel_z,
	       thread_truck_steer,thread_truck_speed;//assigning identifiers to real time threads


extern int current_state;//global variable defining current state

// variables defining current update instants of respective sensor values

extern int current_accelerometer_x,current_accelerometer_y,current_accelerometer_z,
           current_pot,current_speed,current_steering;
 


//thread function declarations

void* truck_potentiometer(void*);//updates trailer cab angle

void* truck_steering(void*);//updates steering angle

void* truck_speed_motor(void*);//updates speed of motor

void* truck_accelerometer_x(void*);// updates truck acceleration truck_accel_x along x axis

void* truck_accelerometer_y(void*);// updates truck acceleration truck_accel_y along y axis

void* truck_accelerometer_z(void*);// updates truck acceleration truck_accel_z along z axis

void* current_state_K(void*)//updates current state

//sensor update access functions. upto 5 previous sensor updates can be accessed by passing a value n to function which is the previous nth update required

float truck_potentiometer_update (int);// returns nth previous update of trailer-cab angle

float truck_steering_update (int);//return nth previous update of truck steering angle

float truck_speed_update (int);//returns nth previous update of truck speed

float truck_accelerometer_x_update (int);//returns nth previous update of acceleration in x direction

float truck_accelerometer_y_update (int);//returns nth previous update of acceleration in y direction

float truck_accelerometer_z_update (int);// returns nth previous update of acceleration in z direction



