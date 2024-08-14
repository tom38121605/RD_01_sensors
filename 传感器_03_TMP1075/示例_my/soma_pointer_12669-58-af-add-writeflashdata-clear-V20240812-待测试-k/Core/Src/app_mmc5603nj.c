  
#include "app_mmc5603nj.h"


//read the mmc5603nj device id
uint8_t mmc5603nj_get_deviceid(void)
{     
   uint8_t iregval=0; 
   
   iregval = i2c_readreg( MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_PRODUCTID1);    
   return iregval;
}  

//sensor init
void mmc5603nj_init(void)
{
   uint8_t iregval=0;

   //read device id
   iregval = mmc5603nj_get_deviceid();

   //check device id
   if(iregval == MMC5603NJ_PRODUCT_ID)
   {
       //software reset
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL1, MMC5603NJ_CMD_SW_RST);
	   LL_mDelay(22);

	   //config control 0 0x1B
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, 0);

	   //config control ODR, set sampling rate
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_ODR, 255);

	   // config control 1 0x1C, Set BW
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL1, MMC5603NJ_CMD_BW11);

	   //config control 2 0x1D, set the continuous mode
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL2, MMC5603NJ_CMD_HPOWER);

   }
   else
         uartsendstr( (uint8_t*)"id err");

}

//config the continuous mode
void mmc5603nj_config_continuous(void)
{
   //config control 0 0x1B, set the functions of freq enable
   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, MMC5603NJ_CMD_CMM_FREQ_EN|MMC5603NJ_CMD_AUTO_SR_EN);

   //config control 2 0x1D, set the continuous mode
   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL2, MMC5603NJ_CMD_CMM_EN|MMC5603NJ_CMD_HPOWER);
}

//read auto data
void mmc5603nj_read_continuous_data(void)
{
   mmc5603nj_get_data(MMC5603NJ_REG_X1,imagdata+2,10);
   mmc5603nj_get_data(MMC5603NJ_DEVICE_STATUS1,imagdata+12,1);
}

//read single data
void mmc5603nj_read_single_data(void)
{
   static uint32_t ireadcount = 0;

   //start sample
   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, 0);
   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, MMC5603NJ_CMD_TMM);

   //read data
   mmc5603nj_get_data(MMC5603NJ_REG_X1,imagdata+2,10);
   mmc5603nj_get_data(MMC5603NJ_DEVICE_STATUS1,imagdata+12,1);

   if(ireadcount++>=1000)
   {
	   ireadcount=0;
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, 0);
	   i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, MMC5603NJ_CMD_TMT|MMC5603NJ_CMD_SET);
   }

}
 
//get the magnetic data
void mmc5603nj_get_data(uint8_t iregaddr, uint8_t *iregval, uint8_t inum)
{ 
   i2c_readbytes(MMC5603NJ_IC_ADDRESS, iregaddr, iregval, inum);
}


