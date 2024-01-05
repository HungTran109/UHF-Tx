/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "sys_ctx.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

#define ADC_CHANNEL_COUNT 4
#define ADC_NUMBER_OF_CONVERSION_TIMES 10
#define ADC_VERSION_INDEX   0
#define ADC_VBAT_INDEX      1
#define ADC_5V_INDEX        2
#define ADC_VREF_INDEX      3

static uint8_t m_adc_in_running = 0;
volatile uint16_t m_adc_raw_data[ADC_CHANNEL_COUNT];
static uint32_t m_adc_channel_info[ADC_CHANNEL_COUNT] =
{
    LL_ADC_CHANNEL_3,   /*Version*/
    LL_ADC_CHANNEL_4,    //Vbat
    LL_ADC_CHANNEL_5, /*Vbus 5V*/
    LL_ADC_CHANNEL_VREFINT,
};
/* USER CODE END 0 */

/* ADC init function */
void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration
  PA3   ------> ADC_IN3
  PA4   ------> ADC_IN4
  PA5   ------> ADC_IN5
  */
  GPIO_InitStruct.Pin = ADC_VERSION_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_VERSION_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_VBAT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_VBAT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_5V_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_5V_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_4);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_5);

  /** Common config
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /* USER CODE BEGIN ADC_Init 2 */
//    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC1);

        /* Poll for ADC effectively calibrated */
        #if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
        #endif /* USE_TIMEOUT */
        
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
        {
        #if (USE_TIMEOUT == 1)
          /* Check Systick counter flag to decrement the time-out value */
          if (LL_SYSTICK_IsActiveCounterFlag())
          {
            if(Timeout-- == 0)
            {
            /* Time-out occurred. Set LED to blinking mode */
            LED_Blinking(LED_BLINK_ERROR);
            }
          }
        #endif /* USE_TIMEOUT */
        }
    }
    
//    LL_ADC_EnableIT_EOC(ADC1);
    // Clear the ADRDY bit in ADC_ISR register by programming this bit to 1.
	SET_BIT(ADC1->ISR, LL_ADC_FLAG_ADRDY);
    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
  /* USER CODE END ADC_Init 2 */

}

/* USER CODE BEGIN 1 */
void adc_isr_cb(void)
{
    /* Check whether ADC group regular end of unitary conversion caused         */
    /* the ADC interruption.                                                    */
    if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
    {
        /* Clear flag ADC group regular end of unitary conversion */
        LL_ADC_ClearFlag_EOC(ADC1);

    }

    /* Check whether ADC group regular overrun caused the ADC interruption */
    if(LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
    {
        //DEBUG_ERROR("ADC overrun\r\n");
        LL_ADC_ClearFlag_OVR(ADC1);
    }
}

static void adc_transfer_to_voltage()
{
    sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage = (m_adc_raw_data[ADC_VBAT_INDEX] * 2 * sys_ctx()->uhf_chip_status.ChipState.adc_measurement.Vdda) / 4095;
    sys_ctx()->uhf_chip_status.ChipState.adc_measurement.HardwareVersionVoltage = (m_adc_raw_data[ADC_VERSION_INDEX] * 2 * sys_ctx()->uhf_chip_status.ChipState.adc_measurement.Vdda) / 4095;
    sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BusVoltage = (m_adc_raw_data[ADC_5V_INDEX] * 2 * sys_ctx()->uhf_chip_status.ChipState.adc_measurement.Vdda) / 4095;
    
    
}
void adc_start_measure(void)
{
    LL_GPIO_ResetOutputPin(EN_ADC_GPIO_Port, EN_ADC_Pin);
    HAL_Delay(1);
    m_adc_in_running = 1;
    for(uint32_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
         m_adc_raw_data[i] = 0;
    }
    for(uint32_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        LL_ADC_REG_SetSequencerChannels(ADC1, m_adc_channel_info[i]);
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
        uint32_t wait_loop_index = ((8 * LL_ADC_DELAY_VREFINT_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);

        while(wait_loop_index != 0)
        {
         //TODO -> Reload watchdog
        //WRITE_REG(IWDG->KR, LL_IWDG_KEY_RELOAD); 
        wait_loop_index--;
        }
        
        if(LL_ADC_REG_IsConversionOngoing(ADC1) == 0)
        {
            LL_ADC_REG_StartConversion(ADC1);
        }
        while(LL_ADC_REG_IsConversionOngoing(ADC1))
        {
                // __WFI();
        }
        m_adc_raw_data[i] += LL_ADC_REG_ReadConversionData12(ADC1);
    }
    // Get avg adc data
//    for (uint32_t i = 0; i < ADC_NUMBER_OF_CONVERSION_TIMES; i++)
//    {
//        m_adc_raw_data[i] /= ADC_NUMBER_OF_CONVERSION_TIMES;
//    }
    sys_ctx()->uhf_chip_status.ChipState.adc_measurement.Vref  = *VREFINT_CAL_ADDR;
    sys_ctx()->uhf_chip_status.ChipState.adc_measurement.Vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(m_adc_raw_data[ADC_VREF_INDEX], LL_ADC_RESOLUTION_12B);
    adc_transfer_to_voltage();
    LL_GPIO_SetOutputPin(EN_ADC_GPIO_Port, EN_ADC_Pin);
}


void adc_stop(void)
{
    if (m_adc_in_running)
    {
        m_adc_in_running = 0;
        //HARDWARE_NTC_POWER_ON(0);
//    //    NVIC_DisableIRQ(ADC1_COMP_IRQn);
//        LL_ADC_DeInit(ADC1);
//        LL_ADC_DisableInternalRegulator(ADC1);
//        LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
        /*Do i have to disable it :P - if power saving is not require -> i suggest just leave it alone*/
    }
}
/* USER CODE END 1 */
