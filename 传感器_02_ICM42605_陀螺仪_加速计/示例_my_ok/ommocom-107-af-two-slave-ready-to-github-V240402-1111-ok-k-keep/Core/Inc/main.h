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

#include "stm32c0xx_ll_crc.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_spi.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_usart.h"
#include "stm32c0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define switch2read(x)  x->CR1 &=~(1<<3);  x->CR1 |=  (1<<2);   //TE=0, RE=1
#define switch2send(x)  x->CR1 &=~(1<<2);  x->CR1 |=  (1<<3);   //TE=1, RE=0



#define CMDHEADER1 0xAA 
#define CMDHEADER2 0x55 
#define CMDEND1    0x0D 
#define CMDEND2    0x0A 

#define PACKAGELEN  64 

//#define TXDATALEN  33   
//#define RXDATALEN  69   
 
#define TXDATALEN  34   
#define RXDATALEN  (PACKAGELEN+6)  


#define WAIT2TE_MASTER 300  
#define WAIT2RE0_MASTER 320 //10    
#define WAIT2RE_MASTER 320   
 
#define WAIT2TE_SLAVE   10
#define WAIT2RE_SLAVE   320 


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[RXDATALEN+2];
extern volatile uint8_t irxdata2[RXDATALEN+2];
extern volatile uint32_t irxcount;
extern volatile uint32_t irxcount2;

extern volatile uint8_t flg_timer1;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void delay_nop(uint32_t nCount);
void iputs1(char *msg);
void iputbytes1(uint8_t *msg, uint32_t ilen);
void Send_Data_To_UART1 (uint8_t c);
uint8_t get_checksum_8(uint8_t *ival, uint32_t inum);
void dowith_uart_rx(void);


uint8_t SPI_transfer(uint8_t data);

//mmc0
uint8_t mmc0_read_byte(uint8_t addr);
void mmc0_write_byte(uint8_t addr,  uint8_t value);
void mmc0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void mmc0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void mmc0_init(void);
void mmc0_read_data(uint8_t *sdata, uint8_t ilen);

//mmc1
uint8_t mmc1_read_byte(uint8_t addr);
void mmc1_write_byte(uint8_t addr,  uint8_t value);
void mmc1_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void mmc1_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void mmc1_init(void);
void mmc1_read_data(uint8_t *sdata, uint8_t ilen);

//icm0
uint8_t icm0_read_byte(uint8_t addr);
void icm0_write_byte(uint8_t addr,  uint8_t value);
void icm0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void icm0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void icm0_init(void);    
void icm0_read_data(uint8_t *sdata, uint8_t ilen); 
   
void uartsendcmd(uint8_t *scmd, uint32_t ilen);   
void savetoflash(void);
void dowithuart(void);
void readsensor(void);

uint32_t byte2uint32(uint8_t *strin, uint32_t *strout, uint32_t ilen);
uint32_t calcu_crc32(uint32_t *idatain, uint32_t len);
uint32_t calcu_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in);
uint32_t stm32_crc32(uint32_t *idatain, uint32_t len);
uint32_t stm32_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nRST_Pin LL_GPIO_PIN_2
#define nRST_GPIO_Port GPIOF
#define IMU_READY_0_Pin LL_GPIO_PIN_0
#define IMU_READY_0_GPIO_Port GPIOA
#define IMU_READY_0_EXTI_IRQn EXTI0_1_IRQn
#define IMU_READY_1_Pin LL_GPIO_PIN_1
#define IMU_READY_1_GPIO_Port GPIOA
#define IMU_READY_1_EXTI_IRQn EXTI0_1_IRQn
#define SPI1_CS2_Pin LL_GPIO_PIN_2
#define SPI1_CS2_GPIO_Port GPIOA
#define SPI1_CS0_Pin LL_GPIO_PIN_3
#define SPI1_CS0_GPIO_Port GPIOA
#define SPI1_CS1_Pin LL_GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define IMU_READY_2_Pin LL_GPIO_PIN_7
#define IMU_READY_2_GPIO_Port GPIOB
#define IMU_READY_2_EXTI_IRQn EXTI4_15_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
 