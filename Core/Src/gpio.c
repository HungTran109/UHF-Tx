/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_PWM_GPIO_Port, LED_PWM_Pin);

  /**/
  LL_GPIO_SetOutputPin(EN_POWER_UHF_GPIO_Port, EN_POWER_UHF_Pin);

  /**/
  LL_GPIO_ResetOutputPin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(EN_ADC_GPIO_Port, EN_ADC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(IR_TX_GPIO_Port, IR_TX_Pin);

  /**/
  LL_GPIO_ResetOutputPin(EN_CHARGEB5_GPIO_Port, EN_CHARGEB5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_PWM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_POWER_UHF_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_POWER_UHF_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_CHARGE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_CHARGE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_ADC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BT_VOL_UP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BT_VOL_UP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BT_VOL_DOWN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BT_VOL_DOWN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BT_MUTE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BT_MUTE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BT_PAIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BT_PAIR_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = IR_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IR_TX_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = IR_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(IR_RX_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_CHARGEB5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_CHARGEB5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = UHF_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(UHF_SCL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = UHF_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(UHF_SDA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void hardware_enable_charge(uint8_t en)
{
    if(en)
    {
        LL_GPIO_SetOutputPin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin);
    }
    else
    {
        LL_GPIO_ResetOutputPin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin);
    }
    return ;
}
void hardware_enable_uhf_power(uint8_t en)
{
    if(en)
    {
        LL_GPIO_ResetOutputPin(EN_POWER_UHF_GPIO_Port, EN_POWER_UHF_Pin);
    }
    else
    {
        LL_GPIO_SetOutputPin(EN_POWER_UHF_GPIO_Port, EN_POWER_UHF_Pin);
    }
    return ;
}
/* USER CODE END 2 */
