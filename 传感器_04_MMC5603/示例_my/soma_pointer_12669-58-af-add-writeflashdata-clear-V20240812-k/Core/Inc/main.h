/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

#include "stm32c0xx_ll_crc.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_usart.h"
#include "stm32c0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"
#include "string.h"        


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define STM32_FLASH_BASE 0x08000000   //bootload hex


#define switch2read(x)  x->CR1 &=~(1<<3);  x->CR1 |=  (1<<2);   //TE=0, RE=1
#define switch2send(x)  x->CR1 &=~(1<<2);  x->CR1 |=  (1<<3);   //TE=1, RE=0


#define CMDHEADER1 0xAA 
#define CMDHEADER2 0x55 

#define PACKAGELEN  64  
#define TXDATALEN  34   
#define RXDATALEN  (PACKAGELEN+6)  


#define WAIT2TE_MASTER 300  
#define WAIT2RE0_MASTER 320
#define WAIT2RE_MASTER 320   
 
#define WAIT2TE_SLAVE   10
#define WAIT2RE_SLAVE   320 


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

 
extern volatile uint8_t flg_timer1;

extern uint8_t imagdata[13];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
 

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void delay_nop(uint32_t iCount);
void Send_Data_To_UART1 (uint8_t c);
void iputs1(char *msg);
void iputbytes1(uint8_t *msg, uint32_t ilen);
   
void uartsendcmd(uint8_t *scmd, uint32_t ilen);   
void uartsendstr(uint8_t *str1);
void uart1_rx_callback(void);
void dowithuart(void);

 
uint8_t get_checksum_8(uint8_t *ival, uint32_t inum);
uint32_t byte2uint32(uint8_t *strin, uint32_t *strout, uint32_t ilen);
uint32_t calcu_crc32(uint32_t *idatain, uint32_t len);
uint32_t calcu_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in);
uint32_t stm32_crc32(uint32_t *idatain, uint32_t len);
uint32_t stm32_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in);


uint8_t i2c_writereg(uint16_t idevaddr, uint8_t ireg,uint8_t data);
uint8_t i2c_readreg(uint16_t idevaddr,uint8_t ireg);
uint8_t i2c_writebytes(uint16_t idevaddr, uint8_t ireg, uint8_t *sbuf,uint16_t ilen);
uint8_t i2c_readbytes(uint16_t idevaddr,uint8_t ireg,uint8_t *sbuf,uint16_t ilen);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nRST_Pin LL_GPIO_PIN_2
#define nRST_GPIO_Port GPIOF
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
