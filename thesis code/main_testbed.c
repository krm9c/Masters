# include "testbedheader.h"

int main (){
	
Motor motor0; 
Trajectory trajx; 
trajx.Position=0;
trajx.velocity=1000;
trajx.Acceleration=100;
LoadTrajectory(&motor0,&trajx);

AssignMotor(&motor0,0xdc00,0);











}