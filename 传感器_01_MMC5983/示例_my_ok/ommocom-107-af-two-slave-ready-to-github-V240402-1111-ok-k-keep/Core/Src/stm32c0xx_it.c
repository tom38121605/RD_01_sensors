/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file    stm32c0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32c0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"  //add for flg_exint
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
volatile uint8_t flg_rx=0;
volatile uint8_t irxdata[RXDATALEN+2]={0};
volatile uint8_t irxdata2[RXDATALEN+2]={0};

//volatile uint8_t icurrentindex=0;

volatile uint32_t irxcount = 0;
volatile uint32_t irxcount2 = 0;


/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "string.h"      //add for memcpy

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
   {
   }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32c0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0_RISING */
    flg_exint=1;  
     
     //Send_Data_To_UART1(0x36); //test

    /* USER CODE END LL_EXTI_LINE_0_RISING */
  }
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1_RISING */

    /* USER CODE END LL_EXTI_LINE_1_RISING */
  }
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_7) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_7);
    /* USER CODE BEGIN LL_EXTI_LINE_7_RISING */

    /* USER CODE END LL_EXTI_LINE_7_RISING */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
	  /* Check whether update interrupt is pending */
	  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
	  {
	    /* Clear the update interrupt flag */
	    LL_TIM_ClearFlag_UPDATE(TIM1);
	  }

	  /* TIM1 update interrupt processing */
	  TimerUpdate_Callback();
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles SPI1 interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 interrupt.
  */
volatile uint32_t ilen=0;
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
   static uint8_t flg_rxstart1 =0;
   static uint8_t flg_rxstart2 =0; 
   static uint8_t flg_rxstart3 =0; 
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
      
      __IO uint32_t iregval;
      iregval = LL_USART_ReadReg(USART1, ISR);

      if (iregval & LL_USART_ISR_NE)  //2
      {
         uart_err = 1;
         LL_USART_ClearFlag_NE(USART1);
      }
      else if (iregval & LL_USART_ISR_PE)  //0
      {
         uart_err = 2;
         LL_USART_ClearFlag_PE(USART1);
      }
      else if (iregval & LL_USART_ISR_ORE)  //3
      {
         uart_err = 3;
         LL_USART_ClearFlag_ORE(USART1);
      }
      else if (iregval & LL_USART_ISR_FE)  //1
      {
         uart_err = 4;
         LL_USART_ClearFlag_FE(USART1);
      }

      LL_USART_ClearFlag_RTO(USART1);  //11
      LL_USART_ClearFlag_UDR(USART1);;  //13
      
	}
	 
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}



/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
