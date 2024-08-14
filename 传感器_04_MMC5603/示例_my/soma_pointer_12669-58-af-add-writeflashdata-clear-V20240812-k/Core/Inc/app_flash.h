 
#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__

#include "main.h"

extern uint32_t iflashaddr;

#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Base @ of Page 5, 2 Kbytes */
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Base @ of Page 6, 2 Kbytes */
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Base @ of Page 7, 2 Kbytes */
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) /* Base @ of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) /* Base @ of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) /* Base @ of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) /* Base @ of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) /* Base @ of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Base @ of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000)  


#define FLASH_BASE_ADDR          ((uint32_t)0x08000000)  
#define FLASH_PAGESIZE           ((uint32_t)0x800)           //2K
#define MAX_PAGESIZE              16

#define FLASH_BOOTLOAD_START_PAGE      0  

#define FLASH_APP_START_PAGE         6  
#define FLASH_APP_START_ADDR  ( (uint32_t)(FLASH_BASE_ADDR + FLASH_APP_START_PAGE * FLASH_PAGESIZE) )  //3000

#define FLASH_BOOTFLAG_START_PAGE       14  
#define FLASH_BOOTFLAG_START_ADDR  ( (uint32_t)(FLASH_BASE_ADDR + FLASH_BOOTFLAG_START_PAGE * FLASH_PAGESIZE) )  //7000

#define FLASH_EEPROM_START_PAGE         15   
#define FLASH_EEPROM_START_ADDR    ( (uint32_t)(FLASH_BASE_ADDR + FLASH_EEPROM_START_PAGE * FLASH_PAGESIZE) )   //7800

#define MAX_EEPROMDATALEN    ( (MAX_PAGESIZE- FLASH_EEPROM_START_ADDR)*FLASH_PAGESIZE )   //1*0x800
#define MAX_BINFILEDATALEN   ( (FLASH_BOOTFLAG_START_PAGE- FLASH_APP_START_PAGE)*FLASH_PAGESIZE )  //(14-6)*0x800
 
 
void erease_flash_data(void);
void write_flash_data(uint8_t *idata, uint32_t ilen);


#endif /* __APP_FLASH_H__ */

