#include "Customer.h"

void Delay_Ms(int cnt)
{
	/*  cnt is the time to wait */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	
	return;
}
void Delay_Us(int cnt)
{
	/*  cnt is the time to wait */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	
	return;
}
int I2C_Write_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char cmd)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to wtite
	 * cmd is the value that need to be written to the register
	 * I2C operating successfully, return 1, otherwise return 0;
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	 
	
	return 1;
}
int I2C_Read_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to read
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	 	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	
	 
	return 1;
}
int I2C_MultiRead_Reg(unsigned char i2c_add, unsigned char reg_add, int num, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the first register address to read
	 * num is the number of the register to read	
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	
	return 1;
}

