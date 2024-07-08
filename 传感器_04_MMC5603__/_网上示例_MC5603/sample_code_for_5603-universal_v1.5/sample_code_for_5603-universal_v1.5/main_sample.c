#include "Customer.h"
#include "MMC5603.h" 

float magnetic_field_x;
float magnetic_field_y;
float magnetic_field_z;

int main(void)
{
	/* Magnetic field vector, unit is gauss */
	float mag_raw_data[3] = {0.0};	
	
	/* Enable the sensor. */
	MMC5603_Enable();	
	
	while(1)
	{		
		/* Get the MMC5603 data, unit is gauss */
		MMC5603_GetData(mag_raw_data);	

		magnetic_field_x = mag_raw_data[0];	//unit is gauss
		magnetic_field_y = mag_raw_data[1];	//unit is gauss
		magnetic_field_z = mag_raw_data[2];	//unit is gauss
		
		/* Sampling interval is 20ms, and the sampling rate is 50Hz. */
		Delay_Ms(20);
	}
}
