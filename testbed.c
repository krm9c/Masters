#include<header.h>

float truck_accelerometer_x()
{
	float truck_accel_x;
	aim104_multi_io_ADC(x_adc,3,1);//x_adc is ADC output across channel3(xout of accelerometer)
	truck_accel_x = ((x_adc*3.3/4095)-1.66)/0.333;//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return truck_accel_x;//truck acceleration along x axis
}
	
float truck_accelerometer_y()
{
	float truck_accel_y;
	aim104_multi_io_ADC(y_adc,4,1);//y_adc is ADC output across channel4(yout of accelerometer)
	truck_accel_y = ((y_adc*3.3/4095)-1.66)/0.333;//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return truck_accel_y;//truck acceleration along y axis
}
	
float truck_accelerometer_z()
{
	float truck_accel_z;
	aim104_multi_io_ADC(z_adc,5,1);//z_adc is ADC output across channel5(zout of accelerometer)
	truck_accel_z = ((z_adc*3.3/4095)-1.66)/0.333;//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return truck_accel_z;//truck acceleration along z axis
}

float truck_tilt_x()
{
	float x_tilt;
	aim104_multi_io_ADC(x_adc,3,1);//x_adc is ADC output across channel 3
	x_tilt = asin(((x_adc*3.3/4095)-1.66)/0.333)//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return x_tilt;//truck tilt along x axis in radians
)

float truck_tilt_y()
{
	float y_tilt;
	aim104_multi_io_ADC(y_adc,4,1);//y_adc is ADC output across channel 4
	y_tilt = asin(((y_adc*3.3/4095)-1.66)/0.333)//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return y_tilt;//truck tilt along y axis in radians
)

float truck_tilt_z()
{
	float z_tilt;
	aim104_multi_io_ADC(z_adc,5,1);//z_adc is ADC output across channel 5
	z_tilt = asin(((z_adc*3.3/4095)-1.66)/0.333)//1.66 is 0g point at 3.3 v and .333 is 								sensitivity in v/g
	return z_tilt;//truck tilt along z axis in radians
)
