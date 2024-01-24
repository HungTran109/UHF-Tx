#ifndef _SYS_CTX_H_
#define _SYS_CTX_H_

#include  "main.h"
#include "app_uhf.h"
#define INVALID_FLASH_FREQUENCY_DATA    0xFFFFFFFF

typedef union
{
    struct 
    {
        uint32_t Frequency;
        uint32_t Channel;
     }flash_data;
    uint32_t raw[2];
}flash_data_t;


typedef struct
{
    struct{
    //uint16_t HardwareVersionVoltage;
    uint16_t BatteryVoltage;
    uint16_t BusVoltage;
    uint16_t Vref;
    uint16_t Vdda;
    }adc_measurement;
    struct
    {
        uint32_t Frequency;
    }uhf_state;
    char HardwareVersion[24];
    char FirmwareVersion[24];
}mcu_state_t;
typedef struct 
{
        struct 
        {
            uint8_t Snr;
            uint8_t Rssi;
            uint32_t Frequency;
            //device_state_t ChipState;
            mcu_state_t ChipState;
            uint8_t Volume;
            uint8_t IsMute;
            uint8_t Is_Enter_Change_Freq_Mode;
        }uhf_chip_status;
        struct 
        {
            uint8_t Enable;
        }ir_status;        
        flash_data_t flash_data;
}sys_ctx_t;

sys_ctx_t *sys_ctx(void);
void sys_ctx_init(void);
#endif
