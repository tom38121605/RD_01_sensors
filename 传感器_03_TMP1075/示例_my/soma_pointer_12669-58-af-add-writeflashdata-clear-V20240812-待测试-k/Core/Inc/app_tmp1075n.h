 
#ifndef __APP_TMP1075N_H__
#define __APP_TMP1075N_H__

#include "main.h"


#define DEVICEADDR_TMP1075N  0x92

#define TMP1075N_TEMP_REG               0x00
#define TMP1075N_CONFIG_REG             0x01
#define TMP1075N_LOW_LIMIT_REG          0x02
#define TMP1075N_HIGH_LIMIT_REG         0x03
#define TMP1075N_DEVICE_REG             0x0F

#define TMP1075N_FREQ_27_5_MS           0x00
#define TMP1075N_FREQ_55_MS             0x01
#define TMP1075N_FREQ_110_MS            0x02
#define TMP1075N_FREQ_220_MS            0x03


#define TMP1075N_TEMP_STEP              (0.0625F)



uint16_t tmp1075n_gettemp(void);
void tmp1075n_setconfig( uint8_t *sbuf, uint8_t ilen);
void tmp1075n_setcontinuous( uint8_t ifreq);
void tmp1075n_setshutdown(void);
   

#endif /* __APP_TMP1075N_H__ */

