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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           
#include "string.h"                      
#include "stm32c0xx_hal_flash.h"

#include "app_mmc5603nj.h"
#include "app_tmp1075n.h"
#include "app_flash.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define LOCALSLAVEADDR  1
uint8_t islaveaddrin=0;

uint8_t slavecmd[TXDATALEN]={0}; 


uint32_t idataoff =0;
uint32_t irestlen =0;

#define PAGE_DATA_SIZE  256  //2048             
uint32_t icrctempdata[PAGE_DATA_SIZE/4+1]={0}; 

uint8_t FLASHTEMPDATA[PAGE_DATA_SIZE]= {0}; 
uint32_t ialldatalen=0;
uint32_t istepdatalen=0;
uint8_t flg_haverest =0;

uint8_t imagdata[13]={0};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile uint8_t flg_rx=0;
volatile uint8_t irxdata[RXDATALEN+2]={0};
volatile uint8_t irxdata2[RXDATALEN+2]={0};
volatile uint32_t irxcount = 0;
volatile uint32_t irxcount2 = 0;  

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

 

//static FLASH_EraseInitTypeDef EraseInitStruct;
//uint32_t iflashaddr = 0;
//uint32_t PageError = 0;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_nop(uint32_t iCount) 
{ 

   while( iCount--)  
   { 
      __NOP(); 
   } 
}


void Send_Data_To_UART1 (uint8_t c)
{
   USART1->TDR = c;
   while( (USART1-> ISR & (1<<7) ) ==0 );  //USART_ISR_TC  //USART_ISR_TXE_TXFNF
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

void uartsendstr(uint8_t *str1)
{
   //switch to tx 
   delay_nop(WAIT2TE_SLAVE); //delay_nop(WAIT2TE_MASTER);  
   switch2send(USART1); 

   iputs1((char*)str1);   

   //switch to rx
   delay_nop(WAIT2RE_SLAVE);  //delay_nop(WAIT2RE0_MASTER);    
   switch2read(USART1);   
}   
 
void uart1_rx_callback(void) 
{
      static uint8_t flg_rxstart1 =0;
      static uint8_t flg_rxstart2 =0; 
      static uint8_t flg_rxstart3 =0; 
      static uint8_t ilen =0; 

      uint8_t cuartbyte=0;

      if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1)) 
      {   
         //USART1->ISR &= ~(USART_ISR_RXNE_RXFNE);  //auto cleared by readed
         
         cuartbyte = (uint8_t)(READ_BIT(USART1->RDR, USART_RDR_RDR) & 0xFFU); 

         if( (flg_rxstart1==0) && ( cuartbyte !=CMDHEADER2) )   
            return;      
            
         if( (flg_rxstart1==0) && ( cuartbyte ==CMDHEADER2) )   
         {     
            flg_rxstart1=1; 
            irxdata[irxcount++]=cuartbyte;
            return;
         }     

         if( (flg_rxstart2==0) && (flg_rxstart1==1) && ( cuartbyte !=CMDHEADER1) )  
         {         
            flg_rxstart1=0;
            irxcount=0;
            return;
         }			
         if( (flg_rxstart2==0) && (flg_rxstart1==1) && ( cuartbyte ==CMDHEADER1) )   
         {         
            flg_rxstart2=1;   
            irxdata[irxcount++]=cuartbyte;
            return;
         }
         
         if( (flg_rxstart2==1) && (flg_rxstart3==0) )  
         {   
            ilen=cuartbyte; 
            irxdata[irxcount++]=cuartbyte;
            flg_rxstart3=1;
           
            return;
         }        
         
         if( flg_rxstart3==1)   
         {        
            irxdata[irxcount++]=cuartbyte;

            if(irxcount>=(ilen+3))
            {
               memcpy((void *)irxdata2,(void *)irxdata,irxcount);
               irxcount2=irxcount;
               flg_rx=1;

               irxcount=0;
               ilen=0;
               flg_rxstart1=0;
               flg_rxstart2=0;		
               flg_rxstart3=0;		            
            }                 
         }
      } 
      else
      {
         
         //__IO uint32_t iregval;
         //iregval = LL_USART_ReadReg(USART1, ISR);

         //if (iregval & LL_USART_ISR_NE)  //2
         //   LL_USART_ClearFlag_NE(USART1);
         //else if (iregval & LL_USART_ISR_PE)  //0
         //   LL_USART_ClearFlag_PE(USART1);
         //else if (iregval & LL_USART_ISR_ORE)  //3
         //   LL_USART_ClearFlag_ORE(USART1);
         //else if (iregval & LL_USART_ISR_FE)  //1
         //   LL_USART_ClearFlag_FE(USART1);

         //LL_USART_ClearFlag_RTO(USART1);  //11
         //LL_USART_ClearFlag_UDR(USART1);;  //13
         
      }   
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
   
   LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}

uint32_t stm32_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in)
{
   
   LL_CRC_SetInitialData(CRC, icrc_in);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}


uint8_t i2c_writereg(uint16_t idevaddr, uint8_t ireg,uint8_t data)
{
   if((HAL_I2C_Mem_Write(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,&data,1,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
   }
}

uint8_t i2c_readreg(uint16_t idevaddr,uint8_t ireg)
{
   uint8_t res;
   HAL_I2C_Mem_Read(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,&res,1,1000);
   return res;
}

uint8_t i2c_writebytes(uint16_t idevaddr, uint8_t ireg, uint8_t *sbuf,uint16_t ilen)
{
   if((HAL_I2C_Mem_Write(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,sbuf,ilen,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
   }
}

uint8_t i2c_readbytes(uint16_t idevaddr,uint8_t ireg,uint8_t *sbuf,uint16_t ilen)
{
   if((HAL_I2C_Mem_Read(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,sbuf,ilen,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
   }
}

void dowithuart(void)
{   
   uint32_t ipackdatalen=0;
   uint32_t icrc=0;
   uint32_t icrclen=0;
   uint32_t icrcin=0;  
   static uint32_t icrcstep=0;
   
   switch (irxdata2[3])   //command
   {
      
      case 0xC1:    


            erease_flash_data();   
            
            //response command D1
            slavecmd[0]=CMDHEADER1;       //0xAA;
            slavecmd[1]=CMDHEADER2;       //0x55;
            slavecmd[2]=0x03;              //len
            slavecmd[3]=0xD1;              //command
            slavecmd[4]=LOCALSLAVEADDR;   //local addr         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);     
            
            iflashaddr = FLASH_EEPROM_START_ADDR;   //7800
            idataoff =0;    
            istepdatalen=0;
            icrcstep=LL_CRC_DEFAULT_CRC_INITVALUE; 
            ialldatalen=0;   
            flg_haverest=0;                
      
            break;
            
      case 0xC2:  
      
            ipackdatalen=irxdata2[2]-3;
      
            if( (ialldatalen+ipackdatalen) > MAX_EEPROMDATALEN)
            {
               uartsendcmd( (uint8_t*)"max data err\r\n", 14);  
               break;               
            }        
      
            if( (idataoff+ipackdatalen) > PAGE_DATA_SIZE)
            {
               uartsendcmd( (uint8_t*)"data err\r\n", 10);  
               break;               
            }
      
            //copy to array
            for( uint32_t i=0; i<ipackdatalen; i++)
            {
               FLASHTEMPDATA[idataoff+i]=irxdata2[5+i];
            }
            
            idataoff += ipackdatalen; 
            
            //------------------save to flash--------------------
            
            if( idataoff == PAGE_DATA_SIZE)
            {      
               flg_haverest=0;
               
               //make step crc 
               istepdatalen=idataoff;
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen);   
               
               //icrc = stm32_crc32(icrctempdata, icrclen);  
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep);

               write_flash_data(FLASHTEMPDATA,istepdatalen);                

               icrcstep=icrc;
               ialldatalen+=istepdatalen;  //idataoff
               idataoff=0;
            }  
            else
            {
               flg_haverest =1;
            } 
            
            //------------------save to flash-----end---------------
            
            
            //response command D2
            slavecmd[0]=CMDHEADER1;  //0xAA;
            slavecmd[1]=CMDHEADER2;  //0x55;
            slavecmd[2]=0x03;       
            slavecmd[3]=0xD2;      
            slavecmd[4]=LOCALSLAVEADDR;         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);                 
            
            break;

      case 0xC3:
         
            //------------------save to flash--------------------
            
            //if( idataoff < PAGE_DATA_SIZE)
            if( flg_haverest==1)
            {                     
               uint8_t iremainder=0;
               
               flg_haverest=0;  

               iremainder= idataoff % 8;   
               if(iremainder!=0)
                   memset(FLASHTEMPDATA+idataoff,0xff,8-iremainder);                
               
               //make step crc 
               istepdatalen=idataoff;
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen);   
               
               //icrc = stm32_crc32(icrctempdata, icrclen);  
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep); 
                
               write_flash_data(FLASHTEMPDATA,istepdatalen);  

               icrcstep=icrc;
               ialldatalen+=istepdatalen;  //idataoff
               idataoff=0;
            }                      
            //------------------save to flash-----end---------------   
      
      
            //check crc
            icrcin+=(uint32_t)(irxdata2[5]<<24);
            icrcin+=(uint32_t)(irxdata2[6]<<16);
            icrcin+=(uint32_t)(irxdata2[7]<<8);
            icrcin+=(uint32_t)(irxdata2[8]);
      
            if(icrcstep==icrcin)   
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

            //--test log
            //uartsendcmd( (uint8_t*)(&ialldatalen), 4);             
            
            //-------------------read out the flash data -------------------            
            
            memset(FLASHTEMPDATA,0,PAGE_DATA_SIZE);  
            iflashaddr = FLASH_EEPROM_START_ADDR;        //0800 3000
            icrcstep=LL_CRC_DEFAULT_CRC_INITVALUE; 
                 
            uint32_t ialldatalen2=0;  
            ialldatalen2=ialldatalen;       
            while( ialldatalen2>0 )    
            {                
               
               if(ialldatalen2 >= PAGE_DATA_SIZE)
                  istepdatalen = PAGE_DATA_SIZE;
               else
                  istepdatalen = ialldatalen2;  
               
                              
               for(uint32_t j=0; j<istepdatalen; j+=4)      
                  *(__IO uint32_t *)( &FLASHTEMPDATA[j] ) = *(__IO uint32_t *)(iflashaddr+j); 
               
               
               //make step crc 
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen); 
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep);  

               //uartsendcmd( FLASHTEMPDATA, istepdatalen);       //--test log             
               
               //ipage++;
               icrcstep=icrc;
               ialldatalen2 -= istepdatalen;
               iflashaddr += istepdatalen;
               istepdatalen = 0;  
               
            }           
                             
            if(icrc!=icrcin) 
                uartsendcmd( (uint8_t*)"crc err\r\n", 9);       

            //-------------------read out the flash data ---end----------------   
            

            break;
            

      default:         
            break;
   } 
       
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   /* USER CODE BEGIN 1 */

   //SCB->VTOR = FLASH_BASE_ADDR + FLASH_APP_START_PAGE * FLASH_PAGESIZE;    //VECT_TAB_OFFSET

   /* USER CODE END 1 */

   /* MCU Configuration--------------------------------------------------------*/

   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* USER CODE BEGIN Init */

   /* USER CODE END Init */

   /* Configure the system clock */
   SystemClock_Config();

   /* USER CODE BEGIN SysInit */


   /* USER CODE END SysInit */

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   MX_USART1_UART_Init();
   MX_TIM1_Init();
   MX_TIM17_Init();
   MX_CRC_Init();
   MX_I2C1_Init();
   /* USER CODE BEGIN 2 */

   uartsendstr( (uint8_t*)"start");

   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */

   //mmc5603 init
   mmc5603nj_init();

   //config mmc5603nj
   mmc5603nj_config_continuous();
   
   while (1)
   { 
      static uint32_t iuart_rxledcount=0;
      static uint32_t isensorcount=0;
      static uint32_t iresetcount=0;
      //uint8_t ichecksum=0;

      //LL_mDelay(100);

      if(flg_timer1==1)
      {
         flg_timer1=0;
       
         iuart_rxledcount++;	
         isensorcount++;
         
         iresetcount++;
      }

      //-------------test read mag data----------------------
      
     if( isensorcount >=10)
      {
         isensorcount=0;

         //read auto data
         //mmc5603nj_read_continuous_data();

         //read single data
         mmc5603nj_read_single_data();

         //print data
         imagdata[0]=0xAA;
         imagdata[1]=0x55;
//         uartsendcmd( imagdata,11); //13
      }


     if( iresetcount >=1)
      {
    	 iresetcount=0;

    	 //do set
    	 i2c_writereg(MMC5603NJ_IC_ADDRESS, MMC5603NJ_REG_CTRL0, MMC5603NJ_CMD_SET);
    	 delay_nop(50);
      }

      //-------------test read mag data---end-------------------

      
      //uart rx
      if (flg_rx==1)
      {
         flg_rx=0;   
         //uartsendstr( (uint8_t*)"rx in\r\n" ); 
         
         islaveaddrin=irxdata2[4];

         //checksum
         uint8_t ichecksum=0;
         ichecksum = get_checksum_8((uint8_t *)irxdata2,irxcount2-1);

         if(ichecksum == irxdata2[irxcount2-1])  
            if(islaveaddrin==LOCALSLAVEADDR)            
               dowithuart();             
         
      }   
 
		if( iuart_rxledcount >=1000)
		{	
			 iuart_rxledcount=0;		 
			 LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
		}
 
  }  

   /* USER CODE END WHILE */

   /* USER CODE BEGIN 3 */

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
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
