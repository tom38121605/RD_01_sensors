/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention

  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           
#include "string.h"           
#include "spi.h"              

#include "stm32c0xx_hal_flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define LOCALSLAVEADDR  1
uint8_t islaveaddrin=0;

uint8_t slavecmd[TXDATALEN]={0}; 

//#define DATA_MMC0_POINT  4
//#define DATA_MMC1_POINT  11
//#define DATA_ICM0_POINT  18
//#define DATA_TIME_POINT  30

#define DATA_MMC0_POINT  5
#define DATA_MMC1_POINT  12
#define DATA_ICM0_POINT  19
#define DATA_TIME_POINT  31


uint32_t idataoff =0;
uint32_t irestlen =0;

#define TOTAL_FLASHDATA_SIZE  2048  //39           
uint32_t icrctempdata[TOTAL_FLASHDATA_SIZE/4+1]={0}; 

uint8_t FLASHTEMPDATA[TOTAL_FLASHDATA_SIZE]= {0}; 
uint32_t ialldatalen=0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint32_t  immc0_ctrl_0=0;
uint32_t  immc1_ctrl_0=0;

//---MMC5983---  

#define MMC5983_XOUT_REG            0x00
#define MMC5983_TOUT_REG            0x07
#define MMC5983_STATUS_REG          0x08
#define MMC5983_CONTROL0_REG        0x09
#define MMC5983_CONTROL1_REG        0x0A
#define MMC5983_CONTROL2_REG        0x0B
#define MMC5983_CONTROL3_REG        0x0C
#define MMC5983_PRODUCT_ID_REG      0x2F

#define MMC5983_PRODUCT_ID_RESULT   0x30

#define MMC5983_MYSTERY_REG 0x29
#define MMC5983_MYSTERY_TC_DISABLE 0x01
#define MMC5983_UNLOCK_REG 0x0F
#define MMC5983_UNLOCK_VAL 0xE1

//MEMSIC values
#define MMC5983_CONTROL0_MAG_MEAS   0x01
#define MMC5983_CONTROL0_TEMP_MEAS  0x02
#define MMC5983_CONTROL0_INT_EN     0x04
#define MMC5983_CONTROL0_SET        0x08
#define MMC5983_CONTROL0_RESET      0x10
#define MMC5983_CONTROL0_AUTOSREN   0x20   //add

#define MMC5983_CONTROL0_INIT       0x00
#define MMC5983_CONTROL0_START_TEMP_CMD  (MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET)
#define MMC5983_CONTROL0_START_CMD  (MMC5983_CONTROL0_MAG_MEAS)

#ifdef OMMO_SENSOR_MEMSIC_MMC5983_SET
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_SET)
#else
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_RESET) //RESET will invert the axis making them right handed
#endif

#define MMC5983_CONTROL1_CHIP_RESET 0x80
#define MMC5983_CONTROL1_BW_100     0x00
#define MMC5983_CONTROL1_BW_200     0x01
#define MMC5983_CONTROL1_BW_400     0x02
#define MMC5983_CONTROL1_BW_800     0x03
#define MMC5983_CONTROL1_INIT       MMC5983_CONTROL1_BW_8

#define MMC5983_CONTROL2_CM_FREQ_1000  0X07    //add
#define MMC5983_CONTROL2_CMM_EN        0X08    //add
 
//---ICM42605---  
// Bank 0
#define ICM_DEVICE_ID                          0x42  

#define ICM42605_DEVICE_CONFIG_REG             0x11    
#define ICM42605_INT_CONFIG_REG                0x14   
#define ICM42605_ACCEL_DATA_X1_REG             0x1F  

#define ICM42605_INT_STATUS_REG                0x2D      

#define ICM42605_PWR_MGMT0_REG                 0x4E     
#define ICM42605_GYRO_CONFIG0_REG              0x4F     
#define ICM42605_ACCEL_CONFIG0_REG             0x50   
#define ICM42605_GYRO_CONFIG1_REG              0x51   

#define ICM42605_INT_CONFIG1_REG               0x64  
#define ICM42605_INT_SOURCE0_REG               0x65  
#define ICM42605_WHO_AM_I_REG                  0x75          

#define ICM42605_REG_BANK_SEL_REG              0x76      

// Bank 4
#define ICM42605_APEX_CONFIG5_REG              0x44   


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


//-------------dataflash-------------

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


#define DATA_64                 ((uint64_t)0x12345678aa345678)


#define FLASH_BASE_ADDR          ((uint32_t)0x08000000)  
#define FLASH_PAGESIZE           ((uint32_t)0x800)           //2K

#define FLASH_START_PAGE         15  
#define FLASH_START_ADDR    ( (uint32_t)(FLASH_BASE_ADDR + FLASH_START_PAGE * FLASH_PAGESIZE) )

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t iflashaddr = 0;
uint32_t PageError = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_nop(uint32_t nCount) 
{ 
   for(; nCount != 0; nCount--)  
   { 
      __NOP(); 
   } 
}

void iputs1(char *msg)
{
   while(*msg)
   Send_Data_To_UART1(*msg++);   
}


void iputbytes1(uint8_t *msg, uint32_t ilen)
{
   while(ilen--)
   Send_Data_To_UART1(*msg++);
}

void Send_Data_To_UART1 (uint8_t c)
{
   USART1->TDR = c;
   while( (USART1-> ISR & (1<<7) ) ==0 );
}


uint8_t get_checksum_8(uint8_t *ival, uint32_t inum)
{
   uint32_t i =0;
   uint8_t iret =0;
   uint16_t isum =0;

   for (i=0; i<inum; i++)
   {
      isum += ival[i]; 
   }

   iret = (uint8_t)(isum & 0xff);

   return iret;
}

uint8_t SPI_transfer(uint8_t data)
{
   // transmit
	LL_SPI_TransmitData8(MMC_SPI, data);
	while(!LL_SPI_IsActiveFlag_TXE(MMC_SPI));

   // receive
	while(!LL_SPI_IsActiveFlag_RXNE(MMC_SPI));
	return LL_SPI_ReceiveData8(MMC_SPI);

}


//---------------------mmc0 module---------------------------

uint8_t mmc0_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   return value;
}

void mmc0_write_byte(uint8_t addr,  uint8_t value)
{
 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);
 
}

void mmc0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   for(int index = 0; index < length; index++)
   {
       buffer[index]  = mmc0_read_byte(addr+index);
   } 
 
}

void mmc0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{
 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
       *idata  = SPI_transfer(0x00);
       idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);
 
}

void mmc0_init(void)
{
   uint8_t ivalue=0;

   //check device id
   ivalue = mmc0_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      //iputs1("mmc0 ok");       
      //flg_mmc0_ready=1;    

      //unfrozen test  
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);      //auto ser/reset, enable interrupt    
      mmc0_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN); //1000Hz, continuous mode
      LL_mDelay(1);   
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc0_ctrl_0=MMC5983_CONTROL0_INIT;	
      mmc0_write_byte(MMC5983_CONTROL2_REG, 0x00);         

      //reset   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET);  //reset
      LL_mDelay(20);        
      
      //set control 0  
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc0_ctrl_0=MMC5983_CONTROL0_INIT;			
       
      //set control 1   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);    //800Hz
    
   }
   else
   {
       //flg_mmc0_ready=0;
       //iputs1("mmc0 err");   
   }     
   
}

void mmc0_read_data(uint8_t *sdata, uint8_t ilen)
{
   static uint32_t ireadcount = 0;
 
   //start sample 
   mmc0_write_byte(MMC5983_CONTROL0_REG, immc0_ctrl_0 & 0x7f); 

   mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_MAG_MEAS);    //sample   

   //read data
   mmc0_readfromspi(MMC5983_XOUT_REG, ilen, sdata);   

   //set/reset + Temp command
   if (ireadcount>=1000)
   {
      ireadcount =0;

      mmc0_write_byte(MMC5983_CONTROL0_REG, immc0_ctrl_0 & 0x7f);            
      mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET ); 
      mmc0_write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_BW_800);    //800Hz   
        
   }        
}    


//---------------------mmc0 module----end-----------------------


//---------------------mmc1 module---------------------------

uint8_t mmc1_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   return value;
}

void mmc1_write_byte(uint8_t addr,  uint8_t value)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);
}

void mmc1_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   //read data
   for(int index = 0; index < length; index++)
   {
      buffer[index]  = mmc1_read_byte(addr+index);
   } 
 
}

void mmc1_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
      *idata  = SPI_transfer(0x00);
      idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);
}
 

void mmc1_init(void)
{
   uint8_t ivalue=0;   

   //check device id
   ivalue = mmc1_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      //iputs1("mmc1 ok");       
      //flg_mmc1_ready=1;   

      //unfrozen  test
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);      //auto ser/reset, enable interrupt    
      mmc1_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN); //1000Hz, continuous mode
      LL_mDelay(1);  
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc1_ctrl_0=MMC5983_CONTROL0_INIT;		 
      mmc1_write_byte(MMC5983_CONTROL2_REG,0x00);       

      //reset   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET);  //reset
      LL_mDelay(20);                    

      //set control 0  
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc1_ctrl_0	= MMC5983_CONTROL0_INIT;		
       
      //set control 1   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);    //800Hz
    
   }
   else
   {
      //flg_mmc1_ready=0;
      //iputs1("mmc1 err");   
   }     
   
}

void mmc1_read_data(uint8_t *sdata, uint8_t ilen)
{
   static uint32_t ireadcount = 0;

   //start sample
   mmc1_write_byte(MMC5983_CONTROL0_REG, immc1_ctrl_0 & 0x7f);  	
   mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_MAG_MEAS);    //sample

   //read data
   mmc1_readfromspi(MMC5983_XOUT_REG, ilen, sdata);

   //set/reset + Temp command
   if (ireadcount>=1000)
   {
      ireadcount =0;    
      mmc1_write_byte(MMC5983_CONTROL0_REG, immc1_ctrl_0 & 0x7f);      
      mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET ); 
      mmc1_write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_BW_800);    //800Hz   
   }   
   
}   


//---------------------mmc1 module----end-----------------------


//---------------------icm0 module---------------------------

uint8_t icm0_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   return value;
}

void icm0_write_byte(uint8_t addr,  uint8_t value)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2); 
}

void icm0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   //read data
   for(int index = 0; index < length; index++)
   {
      buffer[index]  = icm0_read_byte(addr+index);
   } 
 
}

void icm0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
      *idata  = SPI_transfer(0x00);
      idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2); 
}
   

void icm0_init(void)   
{
   uint8_t ivalue=0;

   icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x00);    // set bank0

   //check device id
   ivalue =icm0_read_byte(ICM42605_WHO_AM_I_REG);     //read device id 
   if(ivalue == ICM_DEVICE_ID)  
   {
      //flg_icm0_ready=1;      
      //iputs1("icm0 ok");       

      icm0_write_byte(ICM42605_DEVICE_CONFIG_REG, 0x01 );     //reset   
      LL_mDelay(100);     

      icm0_write_byte(ICM42605_PWR_MGMT0_REG, 0x0f);     // set GYRO and ACC in LN mode    
      LL_mDelay(1);

      icm0_write_byte( ICM42605_GYRO_CONFIG0_REG, 0x06 | (3 << 5) );  //GYRO ODR 1KHz,GYRO FS 250dps   
      icm0_write_byte(ICM42605_ACCEL_CONFIG0_REG, 0x06 | (3 << 5) );   //ACC ODR 1kHz, ACC FS  2g
      icm0_write_byte( ICM42605_GYRO_CONFIG1_REG, 0xD6);              //DLPF BW=5Hz; DLPF Latency=32ms, GYRO UI 2nd Order , DEC2 M2 3nd Order
      icm0_write_byte( ICM42605_INT_CONFIG_REG, 0x03);    //INT1  Push pull, Active high
      icm0_write_byte( ICM42605_INT_CONFIG1_REG, 0x00);    //proper INT1 and INT2 pin operation
      icm0_write_byte(ICM42605_INT_SOURCE0_REG, 0x09);     //UI data ready interrupt routed to INT1, UI AGC ready interrupt routed to INT1

      icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x04);    //set bank 4     
      icm0_write_byte( ICM42605_INT_SOURCE0_REG, 0x8F);    //Defines mounting matrix, chip to device frame
      icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x00);    //set bank0

      ivalue=icm0_read_byte(0x2d);     //INT STATUS
   }
   else
   {
      //flg_icm0_ready=0;
      //iputs1("icm0 err");   
   }     
   
}
 

void icm0_read_data(uint8_t *sdata, uint8_t ilen) 
{
   //--uint8_t ivalue=0;

   icm0_write_byte( 0x76, 0x00);    // set bank0	
   //--ivalue=icm0_read_byte(0x2d);      //INT STATUS

   icm0_readfromspi(0x1f, ilen, sdata);
}   

//---------------------icm0 module----end-----------------------


void uartsendcmd(uint8_t *scmd, uint32_t ilen)
{
   //switch to tx 
   delay_nop(WAIT2TE_SLAVE); //delay_nop(WAIT2TE_MASTER);  
   switch2send(USART1); 

   //iputbytes1((uint8_t *)scmd,ilen);     
   iputbytes1(scmd,ilen);     

   //switch to rx
   delay_nop(WAIT2RE_SLAVE);  //delay_nop(WAIT2RE0_MASTER);    
   switch2read(USART1);   
}   
 
void savetoflash(void)  
{
   uint32_t icrc=0;
   uint32_t icrclen=0;
   uint32_t icrcin=0;   
   uint8_t flg_flasherr=0; 

   //---------------  erease the flash -----------------

   HAL_FLASH_Unlock();

   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
   EraseInitStruct.Page        = FLASH_START_PAGE;          //15, 0800 7800
   EraseInitStruct.NbPages     = 1;                         //1 x 800 

   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
   {
      flg_flasherr=1;    
   }

   //---------------  write buf to flash -----------------

   iflashaddr = FLASH_START_ADDR;   //0800 7800

   for(uint32_t i=0; i<ialldatalen; i+=8)
   {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, iflashaddr, *(uint64_t*)(&FLASHTEMPDATA[i]) ) == HAL_OK)
      {
         iflashaddr += 8;  //move 8 bytes
      }
      else
      {
         flg_flasherr=1; 
      }               
   }           

   HAL_FLASH_Lock();  
   

   if(flg_flasherr==1)
   {
      flg_flasherr=0;
      uartsendcmd( (uint8_t*)"flash err\r\n", 8);   
   }        
   
}   

void dowithuart(void)
{   
   uint16_t itimnum=0;
   uint32_t idatalen=0;
   uint8_t stest[20]={0};   
   uint32_t icrc=0;
   uint32_t icrclen=0;
   uint32_t icrcin=0;  

////   static uint8_t itestcrc=0;   

   
   switch (irxdata2[3])   //command
   {
      case 0xA1:
         
////            slavecmd[0]=CMDHEADER1;    //0xAA;
////            slavecmd[1]=CMDHEADER2;    //0x55;
////            slavecmd[2]=0X1E;         
////            slavecmd[3]=0xB1;          
////         
////            itimnum = LL_TIM_GetCounter(TIM17);
////            slavecmd[ DATA_TIME_POINT] = (uint8_t)(itimnum & 0xff);
////            slavecmd[ DATA_TIME_POINT+1] = (uint8_t)( (itimnum>>8) & 0xff );   
////      
////            slavecmd[32]=get_checksum_8(slavecmd,32);
////            uartsendcmd(slavecmd,33);  
           
            slavecmd[0]=CMDHEADER1;    //0xAA;
            slavecmd[1]=CMDHEADER2;    //0x55;
            slavecmd[2]=0x1F;         
            slavecmd[3]=0xB1;          
            slavecmd[4]=LOCALSLAVEADDR;          
         
            itimnum = LL_TIM_GetCounter(TIM17);
            slavecmd[ DATA_TIME_POINT] = (uint8_t)(itimnum & 0xff);
            slavecmd[ DATA_TIME_POINT+1] = (uint8_t)( (itimnum>>8) & 0xff );   
      
            slavecmd[33]=get_checksum_8(slavecmd,33);
            uartsendcmd(slavecmd,34);  
           
            break;
      
      case 0xC1:  

//            slavecmd[0]=CMDHEADER1;  //0xAA;
//            slavecmd[1]=CMDHEADER2;  //0x55;
//            slavecmd[2]=0x02;       
//            slavecmd[3]=0xD1;      
//            slavecmd[4]=get_checksum_8(slavecmd,4);
//            uartsendcmd(slavecmd,5);      
      
            slavecmd[0]=CMDHEADER1;  //0xAA;
            slavecmd[1]=CMDHEADER2;  //0x55;
            slavecmd[2]=0x03;       
            slavecmd[3]=0xD1;  
            slavecmd[4]=LOCALSLAVEADDR;         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);      
      
            idataoff =0;    
      
            break;

      case 0xC2:  
         
            idatalen=irxdata2[2]-3;
      
            //if( (idataoff+idatalen) > TOTAL_FLASHDATA_SIZE)
            //   iputs1("data err\r\n");
      
            //save to array
            for( uint32_t i=0; i<idatalen; i++)
            {
               FLASHTEMPDATA[idataoff+i]=irxdata2[5+i];
            }
            
            idataoff += idatalen;
      
            //make step crc...... 
      
         
            slavecmd[0]=CMDHEADER1;  //0xAA;
            slavecmd[1]=CMDHEADER2;  //0x55;
            slavecmd[2]=0x03;       
            slavecmd[3]=0xD2;      
            slavecmd[4]=LOCALSLAVEADDR;         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);      
            
            break;

      case 0xC3:
      
            ialldatalen=idataoff;
      
            //make crc 
            icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,ialldatalen);   
            icrc = stm32_crc32(icrctempdata, icrclen);  
      
            //check crc
            icrcin+=(uint32_t)(irxdata2[5]<<24);
            icrcin+=(uint32_t)(irxdata2[6]<<16);
            icrcin+=(uint32_t)(irxdata2[7]<<8);
            icrcin+=(uint32_t)(irxdata2[8]);
      
      
////            if(itestcrc==0)    //test
////            {
////               itestcrc=1;
////               icrc+=1; 
////            }
      
            if(icrc==icrcin)   
            {
               slavecmd[0]=CMDHEADER1;  //0xAA;
               slavecmd[1]=CMDHEADER2;  //0x55;
               slavecmd[2]=0x03;       
               slavecmd[3]=0xD3;      
               slavecmd[4]=LOCALSLAVEADDR;         
               slavecmd[5]=get_checksum_8(slavecmd,5);
               uartsendcmd(slavecmd,6);     
            }  
            else
            {
               slavecmd[0]=CMDHEADER1;  //0xAA;
               slavecmd[1]=CMDHEADER2;  //0x55;
               slavecmd[2]=0x03;       
               slavecmd[3]=0xE3;      
               slavecmd[4]=LOCALSLAVEADDR;         
               slavecmd[5]=get_checksum_8(slavecmd,5);
               uartsendcmd(slavecmd,6);   

               break;               
            }             
            
            //save the uart data to flash
            savetoflash();

            
            //read out the flash data 
            
            memset(FLASHTEMPDATA,0,ialldatalen);  
            
            iflashaddr = FLASH_START_ADDR;        //0800 7800
            
            for(uint32_t i=0; i<ialldatalen; i+=4)            
              *(__IO uint32_t *)( &FLASHTEMPDATA[i] ) = *(__IO uint32_t *)(FLASH_START_ADDR+i);    
                 
            //check the flash crc 
            icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,ialldatalen);   
            icrc = stm32_crc32(icrctempdata, icrclen);  
            
            if(icrc!=icrcin) 
                uartsendcmd( (uint8_t*)"crc err\r\n", 9);       


//---test log ---         
//uartsendcmd( FLASHTEMPDATA, ialldatalen);             

delay_nop(WAIT2TE_SLAVE); 
switch2send(USART1);
Send_Data_To_UART1(icrc>>24);
Send_Data_To_UART1(icrc>>16);
Send_Data_To_UART1(icrc>>8);
Send_Data_To_UART1(icrc>>0);       
delay_nop(WAIT2RE_SLAVE);  
switch2read(USART1);     
             
//uartsendcmd( (uint8_t*)(&icrc), 4); 
            
            break;

      default:         
            break;
   } 
       
}


   
void readsensor(void)
{
   memset(slavecmd,DATA_MMC0_POINT,28);   
   
   mmc0_read_data(slavecmd+DATA_MMC0_POINT,7);    
   mmc1_read_data(slavecmd+DATA_MMC1_POINT,7);  
   icm0_read_data(slavecmd+DATA_ICM0_POINT,12);
} 


uint32_t byte2uint32(uint8_t *strin, uint32_t *strout, uint32_t ilen)
{

   uint32_t i = 0;
   uint32_t iret = 0;

   //get crc adder
   for ( i=0; i<(ilen/4); i++)
   {
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) | (strin[4*i+ 2] << 8) | strin[4*i+3]);
   }

   //last word
   if ( (ilen%4) == 3)
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) | (strin[4*i+ 2] << 8) );
   if ( (ilen%4) == 2) 
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) );
   if ( (ilen%4) == 1)
      strout[i] = (uint32_t)((strin[4*i] << 24) );
   
   
   //get crc len
   if( (ilen%4) ==0)
      iret =  ilen/4;
   else
      iret = ilen/4 +1;
   
   return iret;
}


uint32_t calcu_crc32(uint32_t *idatain, uint32_t len)
{
   uint32_t icrc_poly = 0x04C11DB7;  
   uint32_t icrc_out = 0xffffffff;  

   for(uint32_t i = 0; i < len; i++)
   {
      icrc_out ^= idatain[i];  

      for (uint8_t j = 0; j < 32; j++)
      {
         if (icrc_out & 0x80000000)
            icrc_out = (icrc_out << 1) ^ icrc_poly;
         else
            icrc_out <<= 1;
      }
   }
   
   return (icrc_out);
}


uint32_t calcu_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in)
{
   uint32_t icrc_poly = 0x04C11DB7;  
   uint32_t icrc_out = icrc_in;  

   for(uint32_t i = 0; i < len; i++)
   {
      icrc_out ^= idatain[i];        
      
      //Send_Data_To_UART1(i);  //test

      for (uint8_t j = 0; j < 32; j++)
      {
         if (icrc_out & 0x80000000)
            icrc_out = (icrc_out << 1) ^ icrc_poly;
         else
            icrc_out <<= 1;
      }
   }
   
   return (icrc_out);
}

uint32_t stm32_crc32(uint32_t *idatain, uint32_t len)
{
   register uint32_t data = 0;
   register uint32_t index = 0;
   
   LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}

uint32_t stm32_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in)
{
   register uint32_t data = 0;
   register uint32_t index = 0;
   
   LL_CRC_SetInitialData(CRC, icrc_in);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
   uint8_t stest[20]={0};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA11);
  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA12);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();  //iputs1("start...\r\n");
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_CRC_Init();
  
  /* USER CODE BEGIN 2 */
    
   LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);   //12Mhz = 48Mhz/4


   LL_mDelay(10);  //20
   uartsendcmd( (uint8_t*)"...",3);     //start
   
   //LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);   //--test
   //LL_mDelay(30); 
   //LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin); 
   
   
   LL_SPI_Enable(SPI1);     

   //sensor init 
   mmc0_init();
   mmc1_init();
   icm0_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    
   while (1)
   { 
      static uint32_t iuart_rxledcount=0;
      static uint32_t isensorcount=0;
      static uint32_t iresetcount=0;
      uint8_t ichecksum=0;      

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if(flg_timer1==1)
      {
         flg_timer1=0;
       
         iuart_rxledcount++;	
         isensorcount++;
      }             
      
      //uart rx
      if (flg_rx==1)
      {
         flg_rx=0;   
         //iputs1("rx in\r\n");

         islaveaddrin=irxdata2[4];

         //checksum
         ichecksum = get_checksum_8((uint8_t *)irxdata2,irxcount2-1);

         if(ichecksum == irxdata2[irxcount2-1])  
            if(islaveaddrin==LOCALSLAVEADDR)            
               dowithuart(); 
         
      }   
      
      //read sensor data
      if(isensorcount>=1)   //1* 1ms =1ms
      {
         isensorcount=0;  
         readsensor();   
      }	
      
      if(iresetcount>=1000)
      {
         iresetcount=0;
         
         //do control0 reset
         mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_RESET );   
         mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_RESET );
      }      
 
		if( iuart_rxledcount >=1000)
		{	
			 iuart_rxledcount=0;		 
			 LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
		}
 
  }  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
   /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
