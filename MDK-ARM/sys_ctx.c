#include "sys_ctx.h"

static sys_ctx_t m_sys_ctx;

sys_ctx_t *sys_ctx()
{
    return &m_sys_ctx;
}
void sys_ctx_init(void)
{
//    m_sys_ctx.uhf_chip_status.ChipState = UHF_State_Unpair; /*Default state -> change if there is an value in flash*/
//    m_sys_ctx.uhf_chip_status.Frequency = 0xFFFFFFFF; /*Mark as invalid frequency*/
//    m_sys_ctx.uhf_chip_status.Volume = 0;
//    m_sys_ctx.uhf_chip_status.Confirm_Count = 0;
//    m_sys_ctx.uhf_chip_status.Confirm_Count2 = 0;
//    m_sys_ctx.uhf_chip_status.CurrentChannelNumber = 0;
    m_sys_ctx.uhf_chip_status.Is_Enter_Change_Freq_Mode = false;
    m_sys_ctx.uhf_chip_status.Rssi = 0xFF;
    m_sys_ctx.uhf_chip_status.Snr = 0xFF;
    m_sys_ctx.uhf_chip_status.Frequency = 720000;//778500;
 
    
    m_sys_ctx.flash_data.flash_data.Channel = 0;
    m_sys_ctx.flash_data.flash_data.Frequency = 0;

}

