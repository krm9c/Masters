#include<header.h>

void init_threads(void* adcFD)

{ 
  pthread_t th0,th1,th2,th3,th4,th5,th6; //7 threads declared

//initialize all sensor update threads as real time tasks(high priority) with 4096 bytes stack size priority 0

  rt_task_init(&thread_truck_pot, truck_potentiometer,1, 1024, 0, 0, 0);
  rt_task_init(&thread_truck_steer, truck_steering,1, 1024, 0, 0, 0);
  rt_task_init(&thread_truck_speed, truck_speed_motor,1, 1024, 0, 0, 0);
  rt_task_init(&thread_truck_accel_x, truck_accelerometer_x,1, 1024, 0, 0, 0);
  rt_task_init(&thread_truck_accel_y, truck_accelerometer_y,1, 1024, 0, 0, 0);
  rt_task_init(&thread_truck_accel_z, truck_accelerometer_z,1, 1024, 0, 0, 0);
  rt_task_init(&thread_cuurent_state_k, current_state_k,1, 1024, 0, 0, 0);

// initialize all important control tasks as real time tasks(low priority)with 1024 bytes stack size priority 1

  rt_task_init(&motor_write_command, WriteCommand,1, 1024, 1, 0, 0);
  rt_task_init(&motor_write_data_word,WriteDataWord,1, 1024, 1, 0, 0);
  rt_task_init(&motor_read_data_word,ReadDataWord,1, 1024, 1, 0, 0);
  rt_task_init(&motor_write_long_int,WriteLongint,1, 1024, 1, 0, 0);
  rt_task_init(&motor_read_long_int,ReadLongint,1, 1024, 1, 0, 0);

// initialize all important sensor previous updates tasks as real time tasks(low priority)with 512 bytes stack size priority 2

  rt_task_init(&truck_pot_update, truck_potentiometer_update,1, 512, 2, 0, 0);
  rt_task_init(&truck_steer_update,truck_steering_update,1, 512, 2, 0, 0);
  rt_task_init(&truck_speedsensor_update,truck_speed_update,1, 512, 2, 0, 0);
  rt_task_init(&truck_accel_x_update,truck_accelerometer_x_update,1, 512, 2, 0, 0);
  rt_task_init(&truck_accel_y_update,truck_accelerometer_y_update,1, 512, 2, 0, 0);
  rt_task_init(&truck_accel_z_update,truck_accelerometer_z_update,1, 512, 2, 0, 0);



//initialize position and velocity status read commands as real time tasks(low priority)with 1024 bytes stack size

   rt_task_init(&read_real_position, ReadRealPosition,1, 1024, 1, 0, 0);
   rt_task_init(&read_real_velocity, ReadRealVelocity,1, 1024, 1, 0, 0);

//intialize all motion start and stop functions as real time tasks (low priority)with 512 bytes stack size
  
   rt_task_init(&turn_on_motor,TurnOnMotor,1, 512, 1, 0, 0);
   rt_task_init(&turn_off_motor,TurnOffMotor,1, 512, 1, 0, 0);
   rt_task_init(&stop_smoothly,StopSmoothly,1, 512, 1, 0, 0);
   rt_task_init(&stop_abruptly,StopAbruptly,1, 512, 1, 0, 0);
  

//make all threads periodic with 1ms sampling rate

  rt_task_make_periodic(&thread_truck_pot, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_truck_steer, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_truck_speed, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_truck_accel_x, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_truck_accel_y, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_truck_accel_z, rt_get_time() + period_of_update, period_of_update);
  rt_task_make_periodic(&thread_current_state_k, rt_get_time() + period_of_update, period_of_update);


//create/run all threads
	
  pthread_create(&th0, null, truck_potentiometer,void *adcFD);
  pthread_create(&th1, null, truck_steering, void *adcFD);
  pthread_create(&th2, null, truck_speed_motor, null);
  pthread_create(&th3, null, truck_accelerometer_x, void *adcFD);
  pthread_create(&th4, null, truck_accelerometer_y, void *adcFD);
  pthread_create(&th5, null, truck_accelerometer_z, void *adcFD);
  pthread_create(&th6, null, current_state_k, null);

}
