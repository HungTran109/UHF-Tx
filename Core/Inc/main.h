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
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void app_main_increase_tick(void);
void app_main(void);
uint32_t app_get_tick(void);
void UART_CharReception_Callback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PWM_Pin LL_GPIO_PIN_0
#define LED_PWM_GPIO_Port GPIOC
#define EN_POWER_UHF_Pin LL_GPIO_PIN_13
#define EN_POWER_UHF_GPIO_Port GPIOC
#define ADC_VERSION_Pin LL_GPIO_PIN_3
#define ADC_VERSION_GPIO_Port GPIOA
#define ADC_VBAT_Pin LL_GPIO_PIN_4
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_5V_Pin LL_GPIO_PIN_5
#define ADC_5V_GPIO_Port GPIOA
#define EN_CHARGE_Pin LL_GPIO_PIN_6
#define EN_CHARGE_GPIO_Port GPIOA
#define EN_ADC_Pin LL_GPIO_PIN_12
#define EN_ADC_GPIO_Port GPIOB
#define BT_VOL_UP_Pin LL_GPIO_PIN_13
#define BT_VOL_UP_GPIO_Port GPIOB
#define BT_VOL_DOWN_Pin LL_GPIO_PIN_14
#define BT_VOL_DOWN_GPIO_Port GPIOB
#define BT_MUTE_Pin LL_GPIO_PIN_15
#define BT_MUTE_GPIO_Port GPIOB
#define BT_PAIR_Pin LL_GPIO_PIN_8
#define BT_PAIR_GPIO_Port GPIOA
#define EN_CHARGEB5_Pin LL_GPIO_PIN_5
#define EN_CHARGEB5_GPIO_Port GPIOB
#define UHF_SCL_Pin LL_GPIO_PIN_8
#define UHF_SCL_GPIO_Port GPIOB
#define UHF_SDA_Pin LL_GPIO_PIN_9
#define UHF_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_Status                          HAL_StatusTypeDef
FLASH_Status HAL_FLASH_Erase(uint32_t address);
FLASH_Status HAL_Flash_ProgramWord(uint32_t address, uint32_t data);
//#define GPIO_ReadOutputDataBit              HAL_GPIO_ReadPin
//#define GPIO_ReadInputDataBit               HAL_GPIO_ReadPin
//#define GPIOToggle                          HAL_GPIO_TogglePin
//#define GPIO_WriteBit                      HAL_GPIO_WritePin
//#define GPIO_SetBits                        HAL_GPIO_
//#define GPIO_ResetBits                      gpio_bit_reset
#define BitAction                           bit_status
#define Bit_SET                                SET
#define Bit_RESET                           RESET

#define GPS_UART                (USART2)


#define FLASH_Unlock                         HAL_FLASH_Unlock
#define FLASH_Lock                            HAL_FLASH_Lock
#define FLASH_ErasePage                   HAL_FLASH_Erase
#define FLASH_ProgramWord               HAL_Flash_ProgramWord

#define FLASH_COMPLETE                 HAL_OK

void Delay_ms(uint32_t ms);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
