#include<header.h>

 

//initialize dynamic memory for each sensor update array

float *truck_accel_x = (float*)calloc(6,sizeof(float));
float *truck_accel_y = (float*)calloc(6,sizeof(float));
float *truck_accel_z = (float*)calloc(6,sizeof(float));
float *truck_pot     =	(float*)calloc(6,sizeof(float));
float *truck_steer   =	(float*)calloc(6,sizeof(float));
float *truck_speed   =	(float*)calloc(6,sizeof(float));

//*truck_accel_x,*truck_accel_y,*truck_accel_z,*truck_pot,*truck_steer,*truck_speed are pointers to respective sensor values

static RTIME period_of_update = nano2count(1000*1000);//update period in timer counts of cpu
static RT_TASK thread_truck_pot,thread_truck_accel_x,thread_truck_accel_y,thread_truck_accel_z,
	       thread_truck_steer,thread_truck_speed;//assigning identifiers to real time threads

int current_accelerometer_x,current_accelerometer_y,current_accelerometer_z,
    current_pot,current_speed,current_steering;
 

int current_state;

pthread_mutex_t adc_mutex = PTHREAD_MUTEX_INITIALIZER;

void* truck_accelerometer_x(void *)

 {
        float x_adc
	int i=0;
        rt_make_hard_real_time();
	pthread_mutex_lock(&adc_mutex);
        while(1) {
		  x_adc =aim104_multi_io_ADC(fd1,3,1);
			//x_adc is ADC output across channel3(xout of accelerometer)
	          truck_accel_x[i] = ((x_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g
		  i=i+1;
		  if (i>5) i=0;
		  current_accelerometer_x = i;
                }
       	       rt_task_wait_period(); /*1ms*/
	       pthread_mutex_unlock(&adc_mutex);
        }
	

void* truck_accelerometer_y(void *)

 {
        float y_adc
	int i=0;
        rt_make_hard_real_time();
	pthread_mutex_lock(&adc_mutex);
        while(1) {
		  y_adc =aim104_multi_io_ADC(fd1,4,1);
			//y_adc is ADC output across channel4(yout of accelerometer)
	          truck_accel_y[i] = ((y_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g
		  i=i+1;
		  if (i>5) i=0;
		  current_accelerometer_y = i;
                }
       	       rt_task_wait_period(); /*1ms*/
      	       pthread_mutex_unlock(&adc_mutex);
        }
	
void* truck_accelerometer_z(void *)

 {
        float z_adc
	int i=0;
        rt_make_hard_real_time();
	pthread_mutex_lock(&adc_mutex);
        while(1) {
		  z_adc =aim104_multi_io_ADC(fd1,5,1);
			//z_adc is ADC output across channel5(zout of accelerometer)
	          truck_accel_z[i] = ((z_adc*3.3/4095)-1.66)/0.333;
			//1.66 is 0g point at 3.3 v and .333 is	sensitivity in v/g
		  i=i+1;
		  if (i>5) i=0;
		  current_accelerometer_x = i;
                }
       	       rt_task_wait_period(); /*1ms*/
               pthread_mutex_unlock(&adc_mutex);
        }

void* truck_potentiometer(void *)

 {
        float truck_pot_adc;
	float pi=(22/7);
	int i=0;
        rt_make_hard_real_time();
	pthread_mutex_lock(&adc_mutex);
        while(1) {
		  truck_pot_adc =aim104_multi_io_ADC(fd1,0,1);
			//truck_pot_adc is ADC output across channel0(zout of accelerometer)
	          truck_pot[i] = ((340*truck_pot_adc/4095)-170)*(pi/180);
			//truck cab-trailer angle from -170 deg to 170 deg in radians
		  i=i+1;
		  if (i>5) i=0;
		  current_pot= i;
                }
       	       rt_task_wait_period(); /*1ms*/
	       pthread_mutex_unlock(&adc_mutex);
        }

void* truck_speed_motor(void*);//updates speed of motor

{
        int i=0;
        rt_make_hard_real_time();
        while(1) {
		  truck_speed[i] = ReadRealVelocity(Motor *motorx);
		  i=i+1;
		  if (i>5) i=0;
		  current_speed = i;
                }
       	       rt_task_wait_period(); /*1ms*/
        }
	

void* current_state_K(void *)

 {	static int i,j,k;
	current_state =0;
	 
        rt_make_hard_real_time();
        for (i=0,i<=1000,i++)
		for (j=0,j<=1000,j++)
			for (k=0,k<=1000,k++){
			 current_state=current_state + 1;
			 }                
       	       rt_task_wait_period(); /*1ms*/
        }


float truck_potentiometer_update (int n);// returns nth previous update of trailer-cab angle
{

   if ((current_pot - n)<0) 
      return truck_pot[6+(current_pot-n)];
   else
	return truck_pot[current_pot-n];
}

float truck_steering_update (int n);//return nth previous update of truck steering angle
{

   if ((current_steering - n)<0) 
      return truck_pot[6+(current_steering-n)];
   else
	return truck_pot[current_steering-n];
}

float truck_speed_update (int n);//returns nth previous update of truck speed
{

   if ((current_speed - n)<0) 
      return truck_pot[6+(current_speed-n)];
   else
	return truck_pot[current_speed-n];
}

float truck_accelerometer_x_update (int n);//returns nth previous update of acceleration in x direction
{

   if ((current_accelerometer_x  - n)<0) 
      return truck_pot[6+(current_accelerometer_x -n)];
   else
	return truck_pot[current_accelerometer_x -n];
}

float truck_accelerometer_y_update (int n);//returns nth previous update of acceleration in y direction
{

   if ((current_accelerometer_y  - n)<0) 
      return truck_pot[6+(current_accelerometer_y -n)];
   else
	return truck_pot[current_accelerometer_y -n];
}

float truck_accelerometer_z_update (int n);// returns nth previous update of acceleration in z direction
{

   if ((current_accelerometer_z  - n)<0) 
      return truck_pot[6+(current_accelerometer_z -n)];
   else
	return truck_pot[current_accelerometer_z -n];
}

