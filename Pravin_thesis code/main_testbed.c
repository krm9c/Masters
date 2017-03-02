# include "testbedheader.h"
int main (){
Motor motor0; 
Trajectory trajx; 
trajx.Position=0;
trajx.velocity=1000;
trajx.Acceleration=100;
AssignMotor(&motor0,0xdc00,0);
LoadTrajectory(&motor0,&trajx);
ioperm(port, size, 1);
}