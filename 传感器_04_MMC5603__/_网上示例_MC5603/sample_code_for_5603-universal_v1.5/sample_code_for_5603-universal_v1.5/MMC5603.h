/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
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
 
#ifndef _MMC5603_H
#define _MMC5603_H 

typedef   signed char  int8_t; 		// signed 8-bit number    (-128 to +127)
typedef unsigned char  uint8_t; 	// unsigned 8-bit number  (+0 to +255)
typedef   signed short int16_t; 	// signed 16-bt number    (-32768 to +32767)
typedef unsigned short uint16_t; 	// unsigned 16-bit number (+0 to +65535)
typedef   signed int   int32_t; 	// signed 32-bt number    (-2,147,483,648 to +2,147,483,647)
typedef unsigned int   uint32_t; 	// unsigned 32-bit number (+0 to +4,294,967,295)

#define MMC5603_7BITI2C_ADDRESS		0x30

#define MMC5603_REG_DATA			0x00
#define MMC5603_REG_XL				0x00
#define MMC5603_REG_XH				0x01
#define MMC5603_REG_YL				0x02
#define MMC5603_REG_YH				0x03
#define MMC5603_REG_ZL				0x04
#define MMC5603_REG_ZH				0x05

#define MMC5603_REG_STATUS1			0x18
#define MMC5603_REG_STATUS0			0x19

#define MMC5603_REG_ODR				0x1A
#define MMC5603_REG_CTRL0			0x1B
#define MMC5603_REG_CTRL1			0x1C
#define MMC5603_REG_CTRL2			0x1D

#define MMC5603_REG_X_THD			0x1E
#define MMC5603_REG_Y_THD			0x1F
#define MMC5603_REG_Z_THD			0x20

#define MMC5603_REG_ST_X_VAL		0x27
#define MMC5603_REG_ST_Y_VAL		0x28
#define MMC5603_REG_ST_Z_VAL		0x29

#define MMC5603_REG_PRODUCTID1		0x39

/* Bit definition for control register ODR 0x1A */
#define MMC5603_CMD_ODR_1HZ			0x01
#define MMC5603_CMD_ODR_5HZ			0x05
#define MMC5603_CMD_ODR_10HZ		0x0A
#define MMC5603_CMD_ODR_50HZ		0x32
#define MMC5603_CMD_ODR_100HZ		0x64
#define MMC5603_CMD_ODR_200HZ		0xC8
#define MMC5603_CMD_ODR_255HZ		0xFF

/* Bit definition for control register 0 0x1B */
#define MMC5603_CMD_TMM				0x01
#define MMC5603_CMD_TMT         	0x02
#define MMC5603_CMD_START_MDT		0x04
#define MMC5603_CMD_SET				0x08
#define MMC5603_CMD_RESET			0x10
#define MMC5603_CMD_AUTO_SR_EN		0x20
#define MMC5603_CMD_AUTO_ST_EN		0x40
#define MMC5603_CMD_CMM_FREQ_EN		0x80

/* Bit definition for control register 1 0x1C */
#define MMC5603_CMD_BW00			0x00
#define MMC5603_CMD_BW01			0x01
#define MMC5603_CMD_BW10			0x02
#define MMC5603_CMD_BW11			0x03
#define MMC5603_CMD_ST_ENP			0x20
#define MMC5603_CMD_ST_ENM			0x40
#define MMC5603_CMD_SW_RST			0x80

/* Bit definition for control register 2 0x1D */
#define MMC5603_CMD_PART_SET1		0x00
#define MMC5603_CMD_PART_SET25		0x01
#define MMC5603_CMD_PART_SET75		0x02
#define MMC5603_CMD_PART_SET100		0x03
#define MMC5603_CMD_PART_SET250		0x04
#define MMC5603_CMD_PART_SET500		0x05
#define MMC5603_CMD_PART_SET1000	0x06
#define MMC5603_CMD_PART_SET2000	0x07
#define MMC5603_CMD_EN_PART_SET		0x08
#define MMC5603_CMD_CMM_EN			0x10
#define MMC5603_CMD_INT_MDT_EN		0x20
#define MMC5603_CMD_INT_MD_EN		0x40
#define MMC5603_CMD_HPOWER			0x80

#define MMC5603_PRODUCT_ID			0x10
#define MMC5603_MM_DONE_INT			0x01
#define MMC5603_MT_DONE_INT			0x02
#define MMC5603_MDT_FLAG_INT		0x04
#define MMC5603_ST_FAIL_INT			0x08
#define MMC5603_OTP_READ_DONE		0x10
#define MMC5603_SAT_SENSOR			0x20
#define MMC5603_MM_DONE				0x40
#define MMC5603_MT_DONE				0x80

// 16-bit mode, null field output (32768)
#define	MMC5603_16BIT_OFFSET		32768
#define	MMC5603_16BIT_SENSITIVITY	1024

/**
 * @brief Enable the sensor
 */
void MMC5603_Enable(void);

/**
 * @brief Disable the sensor
 */
void MMC5603_Disable(void);

/**
 * @brief Get sensor data
 * @param mag_out is the magnetic field vector, unit is gauss
 */
void MMC5603_GetData(float *mag_out);

#endif

