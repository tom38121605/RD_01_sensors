
#include "app_tmp1075n.h"

uint16_t tmp1075n_gettemp(void)
{
   uint8_t itempHL[2];
   uint16_t itempval=0;
   
   i2c_readbytes( DEVICEADDR_TMP1075N, TMP1075N_TEMP_REG, itempHL, 2);  
   itempval = ( (uint16_t)(itempHL[0]<<8) | itempHL[1] ) >>4 ;
   
   return itempval;   
}


void tmp1075n_setconfig( uint8_t *sconfig, uint8_t ilen)
{  
   i2c_writebytes( DEVICEADDR_TMP1075N, TMP1075N_CONFIG_REG, sconfig, ilen);       
}


void tmp1075n_setcontinuous(  uint8_t ifreq)
{  
   uint8_t itempval=0;
   itempval=i2c_readreg(DEVICEADDR_TMP1075N, TMP1075N_CONFIG_REG);
   itempval|= 1<<8;
   
   itempval&= ~(1<<13|1<<14);
   itempval|= ifreq<<13;
   
   
   i2c_writereg( DEVICEADDR_TMP1075N, TMP1075N_CONFIG_REG, itempval);       
}

void tmp1075n_setshutdown(void)
{  
   uint8_t itempval=0;
   itempval=i2c_readreg(DEVICEADDR_TMP1075N, TMP1075N_CONFIG_REG);
   itempval&= ~(1<<8);
   
   i2c_writereg( DEVICEADDR_TMP1075N, TMP1075N_CONFIG_REG, itempval);       
}







