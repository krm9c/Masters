



 (DC >89 or so) chips*/

  return inb(motorx->LM629PortLocation);

}



int LM629Busy(Motor *motorx)

{

return ((inb(motorx->LM629PortLocation) & LM629BusyMask) != 0 );

}



int TrajectoryComplete(Motor *motorx)

{

return ((ReadLM629Status(motorx)&(TrajectoryComMask|CommandErrorMask|ExcessivePosMask)) != 0);

}



void WaitForTrajectoryComplete(Motor *motorx)

{

  while (!TrajectoryComplete(motorx)) {;} /*wait*/

}



void WriteCommand(Motor *motorx, char cmd)

{

  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  outb(motorx->LM629PortLocation, cmd);

}



void WriteDataWord(Motor *motorx , unsigned int data)

{

  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  outb((motorx->LM629PortLocation + LM629DataRegOffset), (data >> 8));

  outb((motorx->LM629PortLocation + LM629DataRegOffset), data);

}



unsigned int ReadDataWord(Motor *motorx)

{

unsigned int high, low;

  while (LM629Busy(motorx)) {;} /*wait till ready;*/

  high = (inb(motorx->LM629PortLocation + LM629DataRegOffset) << 8);

  low  = (inb(motorx->LM629PortLocation + LM629DataRegOffset)) & 0x00ff;

  return (high | low);

}



void WriteLongint(Motor *motorx, unsigned long data)

{

  WriteDataWord(motorx, (unsigned int)((data & 0xffff0000)  >> 16) ); /*high int*/

  WriteDataWord(motorx, (unsigned int)(data & 0x0000ffff )); /*low int*/

}



unsigned long ReadLongint(Motor *motorx)

{

unsigned long high;

unsigned long low;

  high = ((unsigned long)ReadDataWord(motorx)) << 16;

  low  = ((unsigned long)ReadDataWord(motorx)) & 0x0000ffff;

return  (high | low);

}



/******** Miscellaneous LM629 read commands ********/

int ReadSignals(Motor *motorx) /*tca changed - removed "signals"*/

{

  WriteCommand(motorx,RdsigsCmd);

  return ReadDataWord(motorx);

}



long ReadIndexPosition(Motor *motorx)

{

  WriteCommand(motorx,RdipCmd);

  return ReadLongint(motorx);

}



long ReadDesiredPosition(Motor *motorx)

{

  WriteCommand(motorx,RDDPCmd);

  return ReadLongint(motorx);

}



long ReadRealPosition(Motor *motorx)

{

  WriteCommand(motorx,RDRPCmd);

  return ReadLongint(motorx);

}



long ReadDesiredVelocity(Motor *motorx)

{

  WriteCommand(motorx,RDDVcmd);

  return ReadLongint(motorx);

}



long ReadRealVelocity(Motor *motorx)

{

  WriteCommand(motorx,RDRVcmd);

  return ReadLongint(motorx);

}



long ReadIntegraterSum(Motor *motorx)

{

  WriteCommand(motorx,RDSUMcmd);

  return ReadLongint(motorx);

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

  outb(theport, RESETcmd);

for(i=0; i<30000; i++);/*wait around*/ 

/*   delay(5);                      /wait for 5 milliseconds*/

  if ((inb(theport) & ResetDataMask) != ResetData ) return 0;

  else return 1;

}



void TurnOnMotor(Motor *motorx)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,TurnOnMotorCmd);

  WriteCommand(motorx,SttCmd);

}



/*these are emergency measures...*/

void TurnOffMotor(Motor *motorx)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,TurnOffMotorCmd);

  WriteCommand(motorx,SttCmd);

}



void StopSmoothly(Motor *motorx)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,StopSmoothlyCmd);

  WriteCommand(motorx,SttCmd);

}



void StopAbruptly(Motor *motorx)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,StopAbruptlyCmd);

  WriteCommand(motorx,SttCmd);

}



/*these reset with the home and index positions to the current position*/

void DefineHome(Motor *motorx)

{

  WriteCommand(motorx,DfhCmd);

  motorx->LastPosition = 0;

}



void SetIndexPosition(Motor *motorx)

{

  WriteCommand(motorx,SipCmd);

}



/*this command starts a pre-loaded trajectory*/

void StartTrajectory(Motor *motorx)

{

  WriteCommand(motorx,SttCmd);

}



/*this command updates the filter coefficients from their (pre-loaded) buffers*/

void UpdateFilter(Motor *motorx)

{

  WriteCommand(motorx,UdfCmd);

}



void Port8Command(Motor *motorx)

{

  WriteCommand(motorx,Port8Cmd);

}



void LoadPosErrForInt(Motor *motorx, int maxerr)

{

  WriteCommand(motorx,LPEICmd);

  WriteDataWord(motorx,maxerr);

}



void LoadPosErrForStop(Motor *motorx, int maxerr)

{

  WriteCommand(motorx,LPESCmd);

  WriteDataWord(motorx,maxerr);

}



void SetBreakpointAbsolute(Motor *motorx, long position)

{

  WriteCommand(motorx,SBpACmd);

  WriteLongint(motorx,position);

}



void SetBreakpointRelative(Motor *motorx, long position)

{

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

  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKPBit);

  WriteDataWord(motorx,kp);

}



void LoadKI(Motor *motorx, unsigned int ki)

{

  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKIBit);

  WriteDataWord(motorx,ki);

}



void LoadKD(Motor *motorx, unsigned int kd)

{

  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadKDBit);

  WriteDataWord(motorx,kd);

}



void LoadIL(Motor *motorx, unsigned int il)

{

  WriteCommand(motorx,LFilCmd);

  WriteDataWord(motorx,motorx->SamplingInterval | LoadILBit);

  WriteDataWord(motorx,il);

}



void LoadTrajectory(Motor *motorx, Trajectory *traj)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadAllTraj);

  WriteLongint(motorx,traj->Acceleration);

  WriteLongint(motorx,traj->Velocity);

  WriteLongint(motorx,traj->Position);

}



void LoadAcceleration(Motor *motorx, unsigned long acceleration)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadAccBit);

  WriteLongint(motorx,acceleration);

}



void LoadVelocity(Motor *motorx, unsigned long velocity)

{

  WriteCommand(motorx,LTrjCmd);

  WriteDataWord(motorx,motorx->MotionMode | LoadVelBit);

  WriteLongint(motorx,velocity);

}



void LoadPosition(Motor *motorx, long position)

{

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

  sinterval = (sinterval << 8 );

  motorx->SamplingInterval = sinterval;

}



void SetMotionMode(Motor *motorx, int mmode)

{

  motorx->MotionMode = mmode;

}



void SetEncoderlines(Motor *motorx, int lines)

{

  motorx->EncoderLines = lines;

}



void SetClockFrequency(Motor *motorx, long frequency)

{

  motorx->ClockFrequency = frequency;

}



void LoadVelocityInRPM(Motor *motorx, float velocity)

{

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



