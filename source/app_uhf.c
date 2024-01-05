//*****************************************************************************
//  File Name: main.c
//  Function:  KT Wireless Mic Transmitter Products Demoboard
//*****************************************************************************
//        Revision History
//  Version    Date        Description
//  V1.0    2016-04-26  ��ʼ�汾
//  V1.1    2017-02-08  ��ʽ�淶������
//  V0.3    2017-04-27  �Ӽ�Ƶ�ʵ�ʱ���Ȱѵ�Ƶ���ˣ�tune��̨���ٻָ�ԭ����Ƶ�����ã�main.c��
//						��̨��һֱ����ͬ�Ĳ�������tunę�������һ��ʱ��󲽽����4����10����main.c��
//*****************************************************************************

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "KT_WirelessMicTxdrv.h"
#include "main.h"
#include "app_uhf.h"
#include <stdio.h>
#include "app_debug.h"
#include "sys_ctx.h"
#include "internal_flash.h"
#include "app_led.h"
#include "gpio.h"
#include "iwdg.h"
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
#ifdef IR_RECEIVE
extern uint16_t IR_Counter;
extern bool Int_Mode_Flag;
extern uint8_t xdata IRDataCode[4];
#endif

bool temp_RF_POW_SW=0; //���书��״̬��ʱ�洢
static void MCU_GetInfo(void);


int app_uhf_transmit(uint32_t* freq)
{
    //uint32_t lCounter = 0x20000;
    if (freq == NULL)
    {
        MCU_GetInfo();
    }
    
    while(!KT_WirelessMicTx_PreInit())
    {

    };
    while(!KT_WirelessMicTx_Init())
    {
    };
    KT_MicTX_Init(freq);
    return 0;
}

void set_echo_level (uint32_t level, uint32_t delay)
{
    app_led_blink(0, 100, 6);
    KT_WirelessMicTx_ECHO(ECHO_ENABLE,level,delay);
}

void KT_MicTX_Init(uint32_t* freq)
{
    if (freq == NULL)
    {
        Memery_Frequency = sys_ctx()->uhf_chip_status.Frequency;
//        Memery_Frequency = 772000;
    }
    else
    {
        //Memery_Frequency = sys_ctx()->uhf_chip_status.Frequency;
        Memery_Frequency = *freq;
    }
    DEBUG_INFO("Tuning to %u\r\n", Memery_Frequency);
    KT_WirelessMicTx_Tune(Memery_Frequency);
    KT_WirelessMicTx_EQGAIN(EQ_25H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_40H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_63H,EQ_GAIN_0dB);

    KT_WirelessMicTx_EQGAIN(EQ_100H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_160H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_250H,EQ_GAIN_0dB);

    KT_WirelessMicTx_EQGAIN(EQ_400H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_630H,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_1KH,EQ_GAIN_0dB);

    KT_WirelessMicTx_EQGAIN(EQ_1K6,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_2K5,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_4KH,EQ_GAIN_0dB);

    KT_WirelessMicTx_EQGAIN(EQ_6K3,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_10K,EQ_GAIN_0dB);
    KT_WirelessMicTx_EQGAIN(EQ_16K,EQ_GAIN_0dB);

    KT_WirelessMicTx_EQSW(EQ_DISABLE);
    KT_WirelessMicTx_ECHO(ECHO_ENABLE,ECHO_RATIO_0,ECHO_DELAY_92ms);
    Delay_ms(200);
        
    KT_MicTX_RFSwitch();
    KT_WirelessMicTx_PASW(PA_ON);

    KT_WirelessMicTx_Pilot_Fdev(PILOT_FDEV_5K);
    KT_WirelessMicTx_Pilot(PILOT_ENABLE);
}


void KT_MicTX_RFSwitch (void)
{
    /**/
    if (/*RF_POW_SW == 0*/1)
    {
        if(Key_RF_POW_flag)//ȥ��
            Delay_ms(100);
        if(/*RF_POW_SW == 0*/1) 
        {
            KT_WirelessMicTx_PAGain(1); //���10dBm������7dBm
        }
    }
    else
    {
        if(Key_RF_POW_flag)//ȥ��
            Delay_ms(100); 
        if (/*RF_POW_SW == 1*/0)    
        {
            KT_WirelessMicTx_PAGain(42); //���18dBm������15dBm
        }
    }
    temp_RF_POW_SW = 0;
}

void set_active_freq (uint32_t freq)
{
    if (freq > 600000 && (freq <800000)){
        if (freq == 772000)
        {
            app_led_blink(0, 100, 5);
        }
        uint16_t pilotSave;
        Key_RF_POW_flag = 0;
        pilotSave = KT_Bus_Read(0x1F);	  //����Ƶ������
        KT_WirelessMicTx_Pilot(PILOT_DISABLE);
        KT_WirelessMicTx_PAGain(0);
        KT_WirelessMicTx_PASW(PA_OFF);                        
        Delay_ms(5);
        Memery_Frequency = freq;
        sys_ctx()->uhf_chip_status.Frequency = Memery_Frequency;
        InternalFlash_WriteConfig();
        KT_WirelessMicTx_Tune(Memery_Frequency);
        KT_MicTX_RFSwitch();
        KT_WirelessMicTx_PASW(PA_ON);
        KT_Bus_Write(0x1F,pilotSave);//�ָ���Ƶ������
        Key_RF_POW_flag = 1;
    }
}

void KT_MicTX_Next_Fre(void) // ��250KHz
{
    
//    uint16_t pilotSave;
//	Key_RF_POW_flag = 0;
//	pilotSave = KT_Bus_Read(0x1F);	  //����Ƶ������
// 	KT_WirelessMicTx_Pilot(PILOT_DISABLE);
//    KT_WirelessMicTx_PAGain(0);
//    KT_WirelessMicTx_PASW(PA_OFF);   
//    DEBUG_INFO ("turn off PA\r\n");
//    Delay_ms(5);
    //Reset uhf by pull down 50us
    hardware_enable_uhf_power(0);
    DEBUG_INFO ("TURN OFF UHF\r\n");
    for (uint8_t i = 0; i < 51; i++)
    {
        HAL_IWDG_Refresh(&hiwdg);
        Delay_ms(1);
    }
    DEBUG_INFO ("TURN ON UHF\r\n");
    hardware_enable_uhf_power(1);
    Memery_Frequency = Memery_Frequency + BAND_STEP;
    if((Memery_Frequency > BAND_TOP) || (Memery_Frequency < BAND_BOTTOM))
	{
        Memery_Frequency = BAND_BOTTOM;
	}
    app_uhf_transmit(&Memery_Frequency);
//    sys_ctx()->uhf_chip_status.Frequency = Memery_Frequency;

//    InternalFlash_WriteConfig();

//    KT_WirelessMicTx_Tune(Memery_Frequency);
//    DEBUG_INFO ("write tune done: %d\r\n", Memery_Frequency);
////    KT_MicTX_RFSwitch();3
//    KT_WirelessMicTx_PAGain(1);
//    KT_WirelessMicTx_PASW(PA_ON);
//	KT_Bus_Write(0x1F,pilotSave);//�ָ���Ƶ������
//    Key_RF_POW_flag = 1;
}


void KT_MicTX_Previous_Fre(void) // ��250KHz
{
    uint16_t pilotSave;
	Key_RF_POW_flag = 0;
	pilotSave=KT_Bus_Read(0x1F);	  //����Ƶ������
	KT_WirelessMicTx_Pilot(PILOT_DISABLE);
    KT_WirelessMicTx_PAGain(0);
    KT_WirelessMicTx_PASW(PA_OFF);                    
    Delay_ms(5);

    Memery_Frequency = Memery_Frequency - BAND_STEP;
    if((Memery_Frequency > BAND_TOP) || (Memery_Frequency < BAND_BOTTOM))
	{
        Memery_Frequency = BAND_TOP;
    }
    sys_ctx()->uhf_chip_status.Frequency = Memery_Frequency;
    InternalFlash_WriteConfig();
    
    KT_WirelessMicTx_Tune(Memery_Frequency);
    
    KT_MicTX_RFSwitch();
    KT_WirelessMicTx_PASW(PA_ON);
    
	KT_Bus_Write(0x1F, pilotSave);//�ָ���Ƶ������
    Key_RF_POW_flag = 1;
}

void KT_MicTX_Mute(bool mute)
{
//    uint16_t reg1C;

//    reg1C = KT_Bus_Read(0x1C);
    if( /*(reg1C & 0x0001) == 0*/mute) 
    {
        KT_WirelessMicTx_MuteSel(AUDIO_MUTE);
        KT_WirelessMicTx_Pilot(PILOT_DISABLE);
    }
    else 
    {
        KT_WirelessMicTx_MuteSel(AUDIO_UNMUTE);
        KT_WirelessMicTx_Pilot(PILOT_ENABLE);
    }
}


/**
  * @brief  Get different information available in the MCU (Device ID, Revision ID & UID)
  * @param  None
  * @retval None
  */
static void MCU_GetInfo(void)
{
    uint8_t aShowDeviceID[30]    = {0};
    uint8_t aShowRevisionID[30]  = {0};
    uint8_t aShowUIDWord0[32]    = {0};
    uint8_t aShowUIDWord1[32]    = {0};
    uint8_t aShowUIDWord2[32]    = {0};
  /* Display Device ID in string format*/
      sprintf((char*)aShowDeviceID,"Device ID = 0x%lX", (unsigned long)LL_DBGMCU_GetDeviceID());
      
      /* Display Revision ID in string format */
      sprintf((char*)aShowRevisionID,"Revision ID = 0x%lX", (unsigned long)LL_DBGMCU_GetRevisionID());

      /* Display UID Word0 */
      sprintf((char*)aShowUIDWord0,"UID Word0 = 0x%lX", (unsigned long)LL_GetUID_Word0());
      
      /* Display UID Word1 */
      sprintf((char*)aShowUIDWord1,"UID Word1 = 0x%lX", (unsigned long)LL_GetUID_Word1());
      
      /* Display UID Word2 */
      sprintf((char*)aShowUIDWord2,"UID Word2 = 0x%lX", (unsigned long)LL_GetUID_Word2());
    
      DEBUG_INFO("%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n", aShowDeviceID
                                                                                 , aShowRevisionID
                                                                                 , aShowUIDWord0
                                                                                 , aShowUIDWord1
                                                                                 , aShowUIDWord2);
     uint16_t last_UUID_word = (uint16_t)LL_GetUID_Word2();
     last_UUID_word = last_UUID_word / 82;
     if(last_UUID_word >= 800)  /*(800-600) /2.5*/
     {
         Memery_Frequency = BAND_BOTTOM + last_UUID_word * BAND_STEP;
     }
     if(Memery_Frequency >= BAND_TOP)
     {
        Memery_Frequency = BAND_TOP;
     }
     DEBUG_INFO("Base Memory frequency:%d\r\n", Memery_Frequency);
     /**/
     
    /*Make device frequency depend on device UUID*/
}

void KT_Enter_STANDBY_Mode (void)
{
    uint16_t regx = KT_Bus_Read(0x03);
//    KT_Bus_Write(0x03, (regx & 0xfffe) | PA_SEL);
    regx = regx | (1 << 15);
    KT_Bus_Write (0x03 ,regx);
    KT_WirelessMicTx_PAGain(0);
    KT_WirelessMicTx_PASW(PA_OFF);
}

void KT_Exit_STANDBY_Mode (void)
{
    uint16_t regx = KT_Bus_Read(0x03);
//    KT_Bus_Write(0x03, (regx & 0xfffe) | PA_SEL);
    regx = regx & (~(1 << 15));
    KT_Bus_Write (0x03 ,regx);
    KT_WirelessMicTx_PASW(PA_ON);
    KT_WirelessMicTx_PAGain(1);
}


