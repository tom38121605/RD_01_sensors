
#include "app_flash.h"

 
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t iflashaddr = 0;
uint32_t PageError = 0;
 

void erease_flash_data(void)
{

   HAL_FLASH_Unlock();

   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
   EraseInitStruct.Page        = FLASH_EEPROM_START_PAGE;      //15,        //7800
   EraseInitStruct.NbPages     = 1;                            //1 x 800   

   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
   {
      uartsendcmd( (uint8_t*)"erase flash err\r\n", 17);     
   } 
   HAL_FLASH_Lock();   

}   

void write_flash_data(uint8_t *idata, uint32_t ilen)
{

   HAL_FLASH_Unlock();               

   for(uint32_t i=0; i<ilen; i+=8)
   {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, iflashaddr, *(uint64_t*)(&idata[i]) ) == HAL_OK)
      {
         iflashaddr += 8;  //move 8 bytes
      }
      else
      {
         uartsendcmd( (uint8_t*)"save flash err\r\n", 16);  
      }               
   }           

   HAL_FLASH_Lock();  

}
