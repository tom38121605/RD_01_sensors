/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


#define TIMER_INTERVAL 1//ms
#define TIMER_TX_INTERVAL 30//ms
#define TIMER_RX_INTERVAL 1000//ms
#define TIMER_LED_INTERVAL 1000//ms
#define TIMER_ERR_INTERVAL 100//ms
#define TIMER_SPI_INTERVAL 1//ms
#define TIMER_UART_REPORT_INTERVAL 1000//ms

extern volatile uint8_t ommo_timer_rx_flag;
extern volatile uint8_t ommo_timer_tx_flag;
extern volatile uint8_t ommo_timer_led_flag;
extern volatile uint8_t ommo_timer_err_flag;
extern volatile uint8_t ommo_timer_spi_flag;
extern volatile uint8_t ommo_timer_uart_report_flag;

extern  volatile uint32_t itimecount;
extern volatile uint8_t timer_1m_cnt[2];

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM17_Init(void);

/* USER CODE BEGIN Prototypes */

extern void TimerUpdate_Callback(void); 

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

