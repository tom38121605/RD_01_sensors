/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright, and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2019
 *****************************************************************************/

 /**
 * @brief
 * This file implement magnetic sensor driver APIs.
 * Modified history: 
 * V1.5: Update to 1 step factory selftest on 20190619
 */

#include "MMC5603.h"
#include "Customer.h"

#include "math.h"

/* Indicate working mode of sensor */
static uint8_t sensor_state = 1;

/* Function declaration */

/**
 * @brief Factory test mode
 */
int MMC5603_Factory_Test_Mode(void);

/**
 * @brief SET operation 
 */
void MMC5603_SET(void);

/**
 * @brief RESET operation 
 */
void MMC5603_RESET(void);

/**
 * @brief OTP read done check
 */
int MMC5603_Check_OTP(void);

/**
 * @brief Check Product ID
 */
int MMC5603_CheckID(void);

/**
 * @brief Auto self-test registers configuration
 */
void MMC5603_Auto_SelfTest_Configuration(void);

/**
 * @brief Auto self-test
 */
int MMC5603_Auto_SelfTest(void);

/*********************************************************************************
* decription: Factory test mode 
*********************************************************************************/
int MMC5603_Factory_Test_Mode(void)
{
	int i;
	uint8_t data_reg[6] ={0};
	uint16_t data_set[3] = {0};
	uint16_t data_reset[3] = {0};	
	uint32_t delta_data[3] = {0};
	
	const uint16_t thr_srst_low = 100;			
	
	/* Write reg 0x1D */
	/* Set Cmm_en bit '0', Disable continuous mode */	
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL2, 0x00);
	
	Delay_Ms(20);
	
	/* Write reg 0x1B */
	/* Set Auto_SR_en bit '0', Disable the function of automatic set/reset */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, 0x00);	
	
	/* Write reg 0x1C, Set BW<1:0> = 00 */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL1, 0x00); 	
				
	/* Do RESET operation */
	MMC5603_RESET();	
	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_TMM);	
	/* Delay 10 ms to finish the TM operation */
	Delay_Ms(10);		
	/* Read register data */
	I2C_MultiRead_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_DATA, 6, data_reg);
	/* Get high 16bits data */
	data_reset[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	//X axis
	data_reset[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);	//Y axis		
	data_reset[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]);	//Z axis 
	
	/* Do SET operation */
	MMC5603_SET();	
	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS,MMC5603_REG_CTRL0,MMC5603_CMD_TMM);	
	/* Delay 10 ms to finish the TM operation */
	Delay_Ms(10);		
	/* Read register data */
	I2C_MultiRead_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_DATA, 6, data_reg);
	/* Get high 16bits data */
	data_set[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	//X axis
	data_set[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);	//Y axis		
	data_set[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]);	//Z axis 	
	
	for(i=0;i<3;i++)
	{
		if(data_set[i] >= data_reset[i])
			delta_data[i] = data_set[i] - data_reset[i];
		else
			delta_data[i] = data_reset[i] - data_set[i];
	}
	
	/* If output < 100lsb, fail*/
	if(delta_data[0]<thr_srst_low && delta_data[1]<thr_srst_low && delta_data[2]<thr_srst_low)
		return -1;	// fail
	
	return 1;	//pass
}

/*********************************************************************************
* decription: SET operation 
*********************************************************************************/
void MMC5603_SET(void)
{	
	/* Write 0x08 to register 0x1B, set SET bit high */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_SET);	
	
	/* Delay to finish the SET operation */
	Delay_Ms(1);
	
	return;	
}

/*********************************************************************************
* decription: RESET operation 
*********************************************************************************/
void MMC5603_RESET(void)
{	
	/* Write 0x10 to register 0x1B, set RESET bit high */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_RESET);
	
	/* Delay to finish the RESET operation */
	Delay_Ms(1);
	
	return;	
}

/*********************************************************************************
* decription: Product ID check
*********************************************************************************/
int MMC5603_CheckID(void)
{
	unsigned char pro_id = 0;
	
	/* Read register 0x39, check product ID */
	I2C_Read_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_PRODUCTID1, &pro_id);	
	if(pro_id != MMC5603_PRODUCT_ID)
		return -1;
	
	return 1;
}

/*********************************************************************************
* decription: Auto self-test registers configuration
*********************************************************************************/
void MMC5603_Auto_SelfTest_Configuration(void)
{
	int i;
	uint8_t reg_value[3];
	int16_t st_thr_data[3]={0};
	int16_t st_thr_new[3]={0};
	
	int16_t st_thd[3]={0};
	uint8_t st_thd_reg[3];		

	/* Read trim data from reg 0x27-0x29 */
	I2C_MultiRead_Reg(MMC5603_7BITI2C_ADDRESS,MMC5603_REG_ST_X_VAL,3,reg_value);	
	for(i=0;i<3;i++)
	{
		st_thr_data[i] = (int16_t)(reg_value[i]-128)*32; 
		if(st_thr_data[i]<0)
			st_thr_data[i] = -st_thr_data[i];
		st_thr_new[i] = st_thr_data[i]-st_thr_data[i]/5;
		
		st_thd[i] = st_thr_new[i]/8;
		if(st_thd[i]>255)
			st_thd_reg[i] = 0xFF;
		else
			st_thd_reg[i] = (uint8_t)st_thd[i];
	}
	/* Write threshold into the reg 0x1E-0x20 */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_X_THD, st_thd_reg[0]);
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_Y_THD, st_thd_reg[1]);
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_Z_THD, st_thd_reg[2]);
	
	return;
}

/*********************************************************************************
* decription: Auto self-test
*********************************************************************************/
int MMC5603_Auto_SelfTest(void)
{
	uint8_t reg_status = 0;
	
	/* Write 0x40 to register 0x1B, set Auto_st_en bit high */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_AUTO_ST_EN);
	
	/* Delay 15ms to finish the selftest process */
	Delay_Ms(15);
	
	/* Read register 0x18, check Sat_sensor bit */
	I2C_Read_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_STATUS1, &reg_status);	
	if((reg_status&MMC5603_SAT_SENSOR))
	{
		return -1;
	}
	
	return 1;
}

/*********************************************************************************
* decription: Continuous mode configuration with auto set and reset
*********************************************************************************/
void MMC5603_Continuous_Mode_With_Auto_SR(uint8_t bandwith, uint8_t sampling_rate)
{
	/* Write reg 0x1C, Set BW<1:0> = bandwith */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL1, bandwith);
	
	/* Write reg 0x1A, set ODR<7:0> = sampling_rate */
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_ODR, sampling_rate); 
	
	/* Write reg 0x1B */
	/* Set Auto_SR_en bit '1', Enable the function of automatic set/reset */
	/* Set Cmm_freq_en bit '1', Start the calculation of the measurement period according to the ODR*/	
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_CMM_FREQ_EN|MMC5603_CMD_AUTO_SR_EN);

	/* Write reg 0x1D */
	/* Set Cmm_en bit '1', Enter continuous mode */	
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL2, MMC5603_CMD_CMM_EN);	
	
	return;
}

/*********************************************************************************
* decription: Do selftest operation periodically
*********************************************************************************/
int MMC5603_Saturation_Checking(void)
{	
	int ret = 0; //1 pass, -1 fail, 0 elapsed time is less 5 seconds

	/* If sampling rate is 50Hz, then do saturation checking every 250 loops, i.e. 5 seconds */
	static int NumOfSamples = 250;
	static int cnt = 0;

	if((cnt++)>=NumOfSamples)
	{
		cnt = 0;
		ret = MMC5603_Auto_SelfTest();
		if(ret == -1)		
		{
			/* Sensor is saturated, need to do SET operation */
			MMC5603_SET();		
		}
		
		/* Do TM_M after selftest operation */
		I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS,MMC5603_REG_CTRL0,MMC5603_CMD_TMM);	
		Delay_Ms(8);
	}

	return ret;
}

/*********************************************************************************
* decription: Auto switch the working mode between Auto_SR and SETonly
*********************************************************************************/
void MMC5603_Auto_Switch(uint16_t *mag)
{
	float mag_out[3];	
	
	mag_out[0] = ((float)mag[0] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	mag_out[1] = ((float)mag[1] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	mag_out[2] = ((float)mag[2] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	
	if(sensor_state == 1)
	{
		/* If X or Y axis output exceed 10 Gauss, then switch to single mode */
		if((fabs(mag_out[0])>10.0f) || (fabs(mag_out[1])>10.0f))
		{	
			sensor_state = 2;
			
			/* Disable continuous mode */	
			I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL2, 0x00);
			Delay_Ms(15);//Delay 15ms to finish the last sampling
			
			/* Do SET operation */
			I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_SET);	
			Delay_Ms(1);//Delay 1ms to finish the SET operation
			
			/* Do TM_M before next data reading */
			I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS,MMC5603_REG_CTRL0,MMC5603_CMD_TMM);	
			Delay_Ms(8);//Delay 8ms to finish the TM_M operation			
		}	
	}
	else if(sensor_state == 2)
	{
		/* If both of X and Y axis output less than 8 Gauss, then switch to continuous mode with Auto_SR */
		if((fabs(mag_out[0])<8.0f) && (fabs(mag_out[1])<8.0f))
		{	
			sensor_state = 1;
			
			/* Enable continuous mode with Auto_SR */
			I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL0, MMC5603_CMD_CMM_FREQ_EN|MMC5603_CMD_AUTO_SR_EN);
			I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL2, MMC5603_CMD_CMM_EN);
		}
		else
		{	
			/* Sensor checking */
			if(MMC5603_Saturation_Checking()==0)
			{			
				/* Do TM_M before next data reading */
				I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS,MMC5603_REG_CTRL0,MMC5603_CMD_TMM);	
			}
		}		
	}
	
	return;	
}

/*********************************************************************************
* decription: Disable sensor continuous mode
*********************************************************************************/
void MMC5603_Disable(void)
{
	/* Write reg 0x1D */
	/* Set Cmm_en bit '0', Disable continuous mode */	
	I2C_Write_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_CTRL2, 0x00);
	
	Delay_Ms(20);
	
	return;
}

/*********************************************************************************
* decription: Enable sensor
*********************************************************************************/
void MMC5603_Enable(void)
{	
	int ret = 0;
	
	/* Inite the sensor state */
	sensor_state = 1;
	
	/* Check product ID */
	ret = MMC5603_CheckID();
	if(ret<0)
		return;	
	
	/* Auto self-test registers configuration */
	MMC5603_Auto_SelfTest_Configuration();
		
	/* Do SET operation */
	MMC5603_SET();		

	/* Work mode setting */
	MMC5603_Continuous_Mode_With_Auto_SR(MMC5603_CMD_BW00, 50);
	
	Delay_Ms(20);
	
	return;
}

/*********************************************************************************
* decription: Read the data register and convert to magnetic field 
*********************************************************************************/
void MMC5603_GetData(float *mag_out)
{
	uint8_t data_reg[6] ={0};
	uint16_t data_temp[3] = {0};
			
	/* Read register data */
	I2C_MultiRead_Reg(MMC5603_7BITI2C_ADDRESS, MMC5603_REG_DATA, 6, data_reg);
	
	/* Get high 16bits data */
	data_temp[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	
	data_temp[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);			
	data_temp[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]); 

	/* Transform to unit Gauss */
	mag_out[0] = ((float)data_temp[0] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY; 
	mag_out[1] = ((float)data_temp[1] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	mag_out[2] = ((float)data_temp[2] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;

	MMC5603_Auto_Switch(data_temp);	
			
	return;
}
