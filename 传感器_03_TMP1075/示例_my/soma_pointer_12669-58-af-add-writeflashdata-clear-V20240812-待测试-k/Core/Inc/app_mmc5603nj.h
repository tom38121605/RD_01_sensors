 
#ifndef __APP_MMC5603NJ_H__
#define __APP_MMC5603NJ_H__ 

#include "main.h"

 
#define MMC5603NJ_IC_ADDRESS		(0x30<<1)


//reg list
#define MMC5603NJ_REG_X1				0x00
#define MMC5603NJ_REG_X2				0x01
#define MMC5603NJ_REG_Y1				0x02
#define MMC5603NJ_REG_Y2				0x03
#define MMC5603NJ_REG_Z1				0x04
#define MMC5603NJ_REG_Z2				0x05
#define MMC5603NJ_REG_X3				0x06
#define MMC5603NJ_REG_Y3				0x07
#define MMC5603NJ_REG_Z3				0x08
#define MMC5603NJ_DEVICE_STATUS1	0x18

#define MMC5603NJ_REG_ODR				0x1A
#define MMC5603NJ_REG_CTRL0			0x1B
#define MMC5603NJ_REG_CTRL1			0x1C
#define MMC5603NJ_REG_CTRL2			0x1D
#define MMC5603NJ_REG_X_THD			0x1E
#define MMC5603NJ_REG_Y_THD			0x1F
#define MMC5603NJ_REG_Z_THD			0x20
#define MMC5603NJ_REG_ST_X_VAL		0x27
#define MMC5603NJ_REG_ST_Y_VAL		0x28
#define MMC5603NJ_REG_ST_Z_VAL		0x29

#define MMC5603NJ_REG_PRODUCTID1		0x39


//Bit for control ODR 0x1A 
#define MMC5603NJ_CMD_ODR_1HZ			0x01
#define MMC5603NJ_CMD_ODR_5HZ			0x05
#define MMC5603NJ_CMD_ODR_10HZ		0x0A
#define MMC5603NJ_CMD_ODR_50HZ		0x32
#define MMC5603NJ_CMD_ODR_100HZ		0x64
#define MMC5603NJ_CMD_ODR_200HZ		0xC8
#define MMC5603NJ_CMD_ODR_255HZ		0xFF

//Bit for control 0 0x1B  
#define MMC5603NJ_CMD_TMM				0x01
#define MMC5603NJ_CMD_TMT         	0x02
#define MMC5603NJ_CMD_START_MDT		0x04
#define MMC5603NJ_CMD_SET				0x08
#define MMC5603NJ_CMD_RESET			0x10
#define MMC5603NJ_CMD_AUTO_SR_EN		0x20
#define MMC5603NJ_CMD_AUTO_ST_EN		0x40
#define MMC5603NJ_CMD_CMM_FREQ_EN		0x80

//Bit for control 1 0x1C  
#define MMC5603NJ_CMD_BW00			0x00
#define MMC5603NJ_CMD_BW01			0x01
#define MMC5603NJ_CMD_BW10			0x02
#define MMC5603NJ_CMD_BW11			0x03
#define MMC5603NJ_CMD_ST_ENP			0x20
#define MMC5603NJ_CMD_ST_ENM			0x40
#define MMC5603NJ_CMD_SW_RST			0x80

//Bit for control 2 0x1D 
#define MMC5603NJ_CMD_PART_SET1		0x00
#define MMC5603NJ_CMD_PART_SET25		0x01
#define MMC5603NJ_CMD_PART_SET75		0x02
#define MMC5603NJ_CMD_PART_SET100		0x03
#define MMC5603NJ_CMD_PART_SET250		0x04
#define MMC5603NJ_CMD_PART_SET500		0x05
#define MMC5603NJ_CMD_PART_SET1000	0x06
#define MMC5603NJ_CMD_PART_SET2000	0x07
#define MMC5603NJ_CMD_EN_PART_SET		0x08
#define MMC5603NJ_CMD_CMM_EN			   0x10
#define MMC5603NJ_CMD_INT_MDT_EN		0x20
#define MMC5603NJ_CMD_INT_MD_EN		0x40
#define MMC5603NJ_CMD_HPOWER			   0x80


#define MMC5603NJ_PRODUCT_ID			0x10
#define	MMC5603NJ_16BIT_OFFSET		32768
#define	MMC5603NJ_16BIT_SENSITIVITY	1024


uint8_t mmc5603nj_get_deviceid(void);
void mmc5603nj_init(void);
void mmc5603nj_config_continuous(void);
void mmc5603nj_read_continuous_data(void);
void mmc5603nj_read_single_data(void);
void mmc5603nj_get_data(uint8_t iregaddr, uint8_t *iregval, uint8_t inum);


#endif

