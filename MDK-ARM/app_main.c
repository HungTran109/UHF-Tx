/*****************************************************************************
 * Includes Directory
 *****************************************************************************/
#include "app_btn.h"
#include "app_led.h"
#include "main.h"
#include "SEGGER_RTT.h"
#include "app_debug.h"
#include "app_uhf.h"
#include "internal_flash.h"
#include "adc.h"
#include "sys_ctx.h"
#include "gpio.h"
#include <stdio.h>
#include "KT_WirelessMicTxdrv.h"
#include "app_cli.h"
#include <string.h>
#include "lwrb.h"
#include "iwdg.h"
#include "tim.h"
/*****************************************************************************
 * MACROS
 *****************************************************************************/
 #define BT_VOL_DOWN_INDEX 0
 #define BT_VOL_UP_INDEX 1
 #define BT_PAIR_INDEX 2
 #define BT_MUTE_INDEX 3
 
 #define HARDWARE_VERSION_1_0_0_BOT_THRESHOLD       1600
 #define HARDWARE_VERSION_1_0_0_TOP_THRESHOLD       1700
 
 #define BUS_VOLTAGE_LOW_THRESHOLD  4300    
 
 #define BATTERY_VOLTAGE_LOW_THRESHOLD         1700
 #define BATTERY_VOLTAGE_HIGH_THRESHOLD         4200
 
 #define HARDWARE_VERSION_STR_1_0_0   "1.0.0"
 #define FIRMWARE_VERSION_STR_1_0_0    "1.0.0"
 /*****************************************************************************
 * PRIVATE DATATYPES
 *****************************************************************************/
//typedef struct /*<I will just leave it as reference>*/
//{
//    const GPIO_TypeDef *Port;
//    const uint32_t Pin;
//    const uint32_t Index; /*Index in the array -> easy to get var :P*/
//}button_t;
typedef uint32_t (*p_gpio_read_pin_callback)(void);
typedef void (*p_button_pressed_event_handler)(void *args);
typedef void (*p_button_press_so_long_event_handler)(void *args);
 /*****************************************************************************
 * Forward Declaration
 *****************************************************************************/
static void gpio_set(uint32_t port, uint32_t pin, uint8_t state);
//static void gpio_reset(void);
static void work_tick_1ms(void);
static void work_tick_1s(void);
static void on_btn_pressed(int index, int event, void* p_args);
//static void on_btn_release(int index, int event, void* p_args);
static void on_btn_hold_so_long(int index, int event, void* p_args);
static void on_btn_hold(int index, int event, void* p_args);

static uint32_t gpio_read_bt_vol_down(void);
static uint32_t gpio_read_bt_vol_up(void);
static uint32_t gpio_read_bt_pair(void);
static uint32_t gpio_read_bt_mute(void);


static void button_pressed_vol_down_handler(void *args);
static void button_pressed_vol_up_handler(void *args);
static void button_pressed_pair_handler(void *args);
static void button_pressed_mute_handler(void *args);
static void button_mute_press_so_long_handler(void *args);

void handle_data_to_send_through_IR (uint32_t data);

 /*****************************************************************************
 * PRIVATE Variables
 *****************************************************************************/
app_btn_hw_config_t button_hw_cfg[] =
{
  {0, 0, 0},
  {1, 0, 0},
  {2, 0, 0},
  {3, 0, 0}
};
static volatile uint32_t m_sys_tick = 0;
static volatile uint32_t m_delay_ms = 0;

static volatile uint32_t m_tick_1ms = 0;
static volatile uint32_t m_tick_1s = 0;
static volatile uint32_t m_tick_5s = 0;
static const p_gpio_read_pin_callback m_pin_read[] = 
{
        gpio_read_bt_vol_down,
        gpio_read_bt_vol_up,
        gpio_read_bt_pair,
        gpio_read_bt_mute
};
static const p_button_pressed_event_handler m_button_press_handler[] =
{
    button_pressed_vol_down_handler,
    button_pressed_vol_up_handler,
    button_pressed_pair_handler,
    button_pressed_mute_handler
};
static const p_button_press_so_long_event_handler m_button_press_so_long_handler[] =
{
    NULL,
    NULL,
    NULL,
    button_mute_press_so_long_handler
}; 
 /*****************************************************************************
 * Public API
 *****************************************************************************/
 
/**
    here private code
*/
lwrb_t lwrb_cli;
uint8_t ring_buffer[64];

static bool m_cli_started = false;

void app_cli_init (void)
{
    if (m_cli_started == false)
    {
        m_cli_started = true;
        app_cli_start();
    }
}

 /*Indicator code start*/
void indicator_init()
{
    gpio_callback_t gpio_callback = 
    {
            .set_gpio = gpio_set,

    };
     struct led_unit_cfg led_pwm = 
     { 
        .pin = LED_PWM_Pin,
        .port = 0,
        .led_name = "LED alarm",
        .active_level = 0
     };
     app_led_init(gpio_callback, &led_pwm, 1);
}



static GPIO_TypeDef* gpio_forward(uint32_t port)
{
    switch(port)
    {
        case 0:
            return LED_PWM_GPIO_Port;
        default:
            return LED_PWM_GPIO_Port;
    }
}
static void gpio_set(uint32_t port, uint32_t pin, uint8_t state)
{
    /*<transfer from port to stm32 port>*/
    GPIO_TypeDef* Port;
    Port = gpio_forward(port);
    /*<it is library implementation>*/
    if(state)
    {
        LL_GPIO_SetOutputPin(Port, pin);
    }
    else
    {
        LL_GPIO_ResetOutputPin(Port, pin);
    }
}
 /*Indicator code end*/
static uint32_t gpio_read_bt_vol_down(void)
{
    uint32_t temp_val = 0xff;
    temp_val = !LL_GPIO_IsInputPinSet(BT_VOL_DOWN_GPIO_Port, BT_VOL_DOWN_Pin);
//    if (temp_val){
//        DEBUG_INFO ("vol down: %d\r\n", temp_val);
//    }
    return temp_val;
//    return !LL_GPIO_IsInputPinSet(BT_VOL_DOWN_GPIO_Port, BT_VOL_DOWN_Pin);
}
static uint32_t gpio_read_bt_vol_up(void)
{
    uint32_t temp_val = 0xff;
    temp_val = !LL_GPIO_IsInputPinSet(BT_VOL_UP_GPIO_Port, BT_VOL_UP_Pin);
//    if (temp_val){
//        DEBUG_INFO ("vol up: %d\r\n", temp_val);
//    }
    return temp_val;
//    return !LL_GPIO_IsInputPinSet(BT_VOL_UP_GPIO_Port, BT_VOL_UP_Pin);
}
static uint32_t gpio_read_bt_pair(void)
{
    uint32_t temp_val = 0xff;
    temp_val = !LL_GPIO_IsInputPinSet(BT_PAIR_GPIO_Port, BT_PAIR_Pin);
//    if (temp_val){
//        DEBUG_INFO ("pair: %d\r\n", temp_val);
//    }
    return temp_val;    
//    return !LL_GPIO_IsInputPinSet(BT_PAIR_GPIO_Port, BT_PAIR_Pin);
}
static uint32_t gpio_read_bt_mute(void)
{
    uint32_t temp_val = 0xff;
    temp_val = !LL_GPIO_IsInputPinSet(BT_MUTE_GPIO_Port, BT_MUTE_Pin);
//    if (temp_val){
//        DEBUG_INFO ("mute: %d\r\n", temp_val);
//    }
    return temp_val;    
//    return !LL_GPIO_IsInputPinSet(BT_MUTE_GPIO_Port, BT_MUTE_Pin);
}
/*Button code start*/
static void uhf_volume_down(void)
{
    //sys_ctx()->uhf_chip_status.Volume = sys_ctx()->uhf_chip_status.Volume 
    
    if(sys_ctx()->uhf_chip_status.Volume == 0)
    {
        sys_ctx()->uhf_chip_status.Volume = 0;
    }
    else
    {
        sys_ctx()->uhf_chip_status.Volume = sys_ctx()->uhf_chip_status.Volume - 2;
    }
    uint8_t reg_val = (uint8_t)sys_ctx()->uhf_chip_status.Volume / 2;
    DEBUG_INFO("Volume down: %d\r\n", reg_val);
    KT_WirelessMicTx_PAGain(reg_val);
}
static void uhf_volume_up(void)
{
    sys_ctx()->uhf_chip_status.Volume = sys_ctx()->uhf_chip_status.Volume + 2;
    if(sys_ctx()->uhf_chip_status.Volume >= 100)
    {
        sys_ctx()->uhf_chip_status.Volume = 100;
    }
    uint8_t reg_val = (uint8_t)sys_ctx()->uhf_chip_status.Volume / 2;
    DEBUG_INFO("Volume up: %d\r\n", reg_val);
    KT_WirelessMicTx_PAGain(reg_val);
}

static uint32_t btn_read(uint32_t pin)
{
    if(pin <= 3)
    {
//        DEBUG_INFO ("PIN:%d\r\n", pin);
        if(m_pin_read[pin])
        {
           return m_pin_read[pin]();
        }
        else
        {
            DEBUG_WARN ("PIN %d not intit yet!!\r\n", pin);
            return 0xff;
        }        
    }
    else
    {
        DEBUG_ERROR ("PIN:%d\r\n", pin);
        return 0xff; /**/
    }
}

static void button_initialize(uint32_t button)
{
     return;
}
static void button_init(void)
{

  static app_btn_config_t m_button_config;
  m_button_config.btn_count = 4;
  m_button_config.config = button_hw_cfg;
  m_button_config.btn_initialize = button_initialize;
  m_button_config.btn_read = btn_read;
  m_button_config.get_tick_cb = app_get_tick;
  
  app_btn_initialize(&m_button_config);
  app_btn_register_callback(APP_BTN_EVT_HOLD, on_btn_hold, NULL);
  app_btn_register_callback(APP_BTN_EVT_HOLD_SO_LONG, on_btn_hold_so_long, NULL);
  app_btn_register_callback(APP_BTN_EVT_PRESSED, on_btn_pressed, NULL);
}
/*Implement function pointer*/

static void button_pressed_vol_down_handler(void *args)
{
    DEBUG_INFO("button vol down pressed\r\n");
    if (sys_ctx()->uhf_chip_status.Is_Enter_Change_Freq_Mode == false) uhf_volume_down();
    else 
    {
        LL_GPIO_SetOutputPin(LED_PWM_GPIO_Port, LED_PWM_Pin);
        Delay_ms(50);
        KT_MicTX_Previous_Fre();
    }
}
static void button_pressed_vol_up_handler(void *args)
{
    DEBUG_INFO("button vol up pressed\r\n");
    if (sys_ctx()->uhf_chip_status.Is_Enter_Change_Freq_Mode == false) uhf_volume_up();
    else 
    {
        LL_GPIO_SetOutputPin(LED_PWM_GPIO_Port, LED_PWM_Pin);
        Delay_ms(50);
        KT_MicTX_Next_Fre();
    }
}
static void button_pressed_pair_handler(void *args)
{
    DEBUG_INFO("button pair pressed\r\n");
    sys_ctx()->ir_status.Enable = !sys_ctx()->ir_status.Enable;
    //KT_MicTX_Next_Fre();
    /*Increase frequency*/
}
static void button_pressed_mute_handler(void *args)
{
    DEBUG_INFO("button mute pressed");
    if(sys_ctx()->uhf_chip_status.IsMute  == 0)
    {
        DEBUG_INFO("Mute transmit device\r\n");
        sys_ctx()->uhf_chip_status.IsMute = 1;
        KT_MicTX_Mute(true);
    }
    else
    {
        DEBUG_INFO("Unmute device\r\n");
       sys_ctx()->uhf_chip_status.IsMute = 0;
        KT_MicTX_Mute(false);
    }
}
static void on_btn_pressed(int index, int event, void* p_args)
{
//    DEBUG_INFO ("INDEX:%d\r\n", index);
  
    if(m_button_press_handler[index] != NULL)
    {
//        DEBUG_INFO ("INDEX:%d\r\n", index);
        m_button_press_handler[index](NULL);
    }
    return;
}
static void on_btn_hold(int index, int event, void* p_args)
{
    return;
}
static void button_mute_press_so_long_handler(void *args)
{
    DEBUG_INFO("button mute so long pressed\r\n");
    if (sys_ctx()->uhf_chip_status.Is_Enter_Change_Freq_Mode == false)
    {
        sys_ctx()->uhf_chip_status.Is_Enter_Change_Freq_Mode = true;
        KT_WirelessMicTx_PAGain(0);
        app_led_on(0, LED_ON_FOREVER);
    }
    else
    {
        sys_ctx()->uhf_chip_status.Is_Enter_Change_Freq_Mode = false;
        app_led_off(0);
        hardware_enable_uhf_power(0);
        InternalFlash_WriteConfig();
        
        DEBUG_INFO("System reset\r\n");
        NVIC_SystemReset();
//        app_led_off(0);
    }
    //KT_MicTX_Next_Fre();
    /*Increase frequency*/
}
static void on_btn_hold_so_long(int index, int event, void* p_args)
{
    if(m_button_press_so_long_handler[index] != NULL)
    {
//        DEBUG_INFO ("INDEX:%d\r\n", index);
        m_button_press_so_long_handler[index](NULL);
    }
    return;
}



/*Button code stop*/

void app_main_increase_tick()
{
    m_tick_1ms++;
    m_tick_1s++;
    m_tick_5s++;
    
    ++m_sys_tick;
    if (m_delay_ms)
    {
        --m_delay_ms;
    }
    
}
uint8_t read_from_lwrb [64];

static void work_tick_1ms(void)
{
    if(m_tick_1ms)
    {
        
        uint8_t read_len = 0;
//        uint8_t ch = 0;
        memset (read_from_lwrb, 0, 64);
        read_len = lwrb_read(&lwrb_cli, read_from_lwrb, 64);
        if (read_len)
        {
            for (uint8_t i = 0; i < read_len; i++)
            {
//                app_cli_poll(read_from_lwrb[i]);
//                LL_USART_TransmitData8(USART2, read_from_lwrb[i]);
                DEBUG_RAW ("%c", read_from_lwrb[i]);
            }
            DEBUG_RAW ("\r\n");
        }
//        uint16_t regx = KT_Bus_Read(0x25);
//        regx = regx & (1 << 14);
//        regx = regx >> 14;
//        DEBUG_WARN ("MUTE STATE:%d\r\n", regx);
        app_led_blink_handler();
        app_btn_scan(NULL);
        m_tick_1ms = 0;
    }
}

//static bool led_state = false;

uint32_t sleep_count = 0;
bool is_sleep = false; 
static void work_tick_5s(void)
{    
    if(m_tick_5s > 5000)
    {
        HAL_IWDG_Refresh(&hiwdg);
        
        
//        if (sleep_count++ > 3)
//        {
//            if (!is_sleep)
//            {
//                KT_Enter_STANDBY_Mode();
//                DEBUG_ERROR ("ENTER STANDBY MODE\r\n");
//                is_sleep = !is_sleep;
//            }
//            else
//            {
//                DEBUG_ERROR ("Exit STANDBY MODE\r\n");
//                KT_Exit_STANDBY_Mode();
//                is_sleep = !is_sleep;
//            }
//            sleep_count = 0;
//        }
//        led_state = !led_state;
//        if (!led_state)
//        {
//            app_led_on (0, 500);
//        }
//        else
//        {
//            app_led_off (0);
//        }

        m_tick_5s = 0;
        adc_start_measure();
        DEBUG_INFO("Board voltage monitor\r\nBattery voltage:%d\r\nBus voltage:%d\r\n",
                                sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage,
                                sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BusVoltage);
        /*If the is 5V USB plug in -> consider enable charging*/
        if(sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BusVoltage > BUS_VOLTAGE_LOW_THRESHOLD)
        {
            if(sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage < BATTERY_VOLTAGE_HIGH_THRESHOLD      // ensure not full
               && sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage > BATTERY_VOLTAGE_LOW_THRESHOLD)   // ensure there is battery
            {
                DEBUG_INFO("CHARGING!!\r\n");
                hardware_enable_charge(1);
            }
            else
            {
                DEBUG_INFO("is not being CHARGED!!\r\n");
                hardware_enable_charge(0);
            }
        }
    }
}

static void work_tick_1s(void)
{
    if (m_tick_1s > 1500)
    {
        static uint16_t count_sendtime = 0;
        if (sys_ctx()->ir_status.Enable)
        {
            if (++count_sendtime < 20)
            {
                // Send Frequency through ir
                uint32_t ir_data = sys_ctx()->uhf_chip_status.Frequency;
                DEBUG_INFO ("IR Send: %u\r\n", ir_data);
                handle_data_to_send_through_IR(ir_data);
                // blink led
                if (count_sendtime % 2) app_led_on(0, 50);
            }
            else 
            {
                // Send 20 times then disable ir
                count_sendtime = 0;
                sys_ctx()->ir_status.Enable = false;
            }
        }
        m_tick_1s = 0;
    }
}

static uint32_t rtt_printf(const void *buffer, uint32_t len)
{
    uint8_t *ptr = (uint8_t*)buffer;
    return SEGGER_RTT_Write(0, ptr, len);
}
/**
 * @brief mutex for the log in service interrupt. In this case, mutex mechanism is not importance case 
                i won't do it in interrupt or multi thread
* @param[in]:  do_lock - Consider lock the log or not 
 * @param[in]: timeout - timeout waiting for the mutex
 * @param[iout]: p_args - Pointer to arguments need for handling event
 */
static bool log_protect(bool do_lock, uint32_t timeout_ms)
{
    return true;
}

void sys_increase_tick(void)
{
    ++m_sys_tick;
    if (m_delay_ms)
    {
        --m_delay_ms;
    }
}

void sys_delay_ms(uint32_t ms)
{
    __disable_irq();
    m_delay_ms = ms;
    __enable_irq();
    while (m_delay_ms);
}
uint32_t sys_get_ms(void)
{
    return m_sys_tick;
}

//uint16_t data_receive = 0;
void decode_nec_protocol (uint16_t* data_rev);
/*Im so lazy so i will put this function into main.h header file*/
void app_main(void)
{
    /*Debug initialize */
    app_debug_init(app_get_tick, log_protect);
    app_debug_register_callback_print(rtt_printf);
    InternalFlash_ReadConfig();
    button_init();    /*<Button initalize>*/
    indicator_init(); /*<led inittalize>*/
    app_uhf_transmit();

    adc_start_measure();
    lwrb_init(&lwrb_cli, ring_buffer, 64);
    app_cli_init();
//    if(sys_ctx()->uhf_chip_status.ChipState.adc_measurement.HardwareVersionVoltage > sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage - 100
//       &&sys_ctx()->uhf_chip_status.ChipState.adc_measurement.HardwareVersionVoltage < (sys_ctx()->uhf_chip_status.ChipState.adc_measurement.BatteryVoltage + 100))
//    {
        sprintf(sys_ctx()->uhf_chip_status.ChipState.HardwareVersion, "%s", HARDWARE_VERSION_STR_1_0_0);
        sprintf(sys_ctx()->uhf_chip_status.ChipState.FirmwareVersion, "%s", FIRMWARE_VERSION_STR_1_0_0);
//    }
//    else
//    {
//        sprintf(sys_ctx()->uhf_chip_status.ChipState.HardwareVersion, "%s", "Unknown");
//        sprintf(sys_ctx()->uhf_chip_status.ChipState.FirmwareVersion, "%s", "Unknown");
//    }
    DEBUG_INFO("UHF Lavalier Microphone Transmitter:\r\nHardware version: %s\r\nFirmware version: %s\r\n", 
                            sys_ctx()->uhf_chip_status.ChipState.HardwareVersion,
                            sys_ctx()->uhf_chip_status.ChipState.FirmwareVersion);
    app_led_blink(0, 100, 6);
    while(1)
    {
        /*Doin polling api*/
        //decode_nec_protocol(&data_receive);
        app_cli_poll();
        work_tick_1ms();
        work_tick_1s();
        work_tick_5s();
    }
}
void generate_logic_in_us (uint8_t logic, uint32_t time_in_us)
{
//    uint16_t pulse_number = 0;
//    pulse_number = down_time / 2;
    time_in_us = time_in_us / 26;
    uint32_t timer2_tick = 0;
    for (uint32_t i = 0; i < time_in_us; i++)
    {
        get_timer_reset_now();
        timer2_tick = get_timer_cnt_now();
        if (!logic)
        {
            LL_GPIO_SetOutputPin (IR_TX_GPIO_Port, IR_TX_Pin);
        }
        
        while ((get_timer_cnt_now() - timer2_tick) < (13)); // 13us
        get_timer_reset_now();
        timer2_tick = get_timer_cnt_now();
        if (!logic)
        {
            LL_GPIO_ResetOutputPin (IR_TX_GPIO_Port, IR_TX_Pin);
        }
        while ((get_timer_cnt_now() - timer2_tick) < (13)); // 13us
        if (!logic)
        {
            LL_GPIO_SetOutputPin (IR_TX_GPIO_Port, IR_TX_Pin);
        }
    }
}

void handle_data_to_send_through_IR (uint32_t data)
{
    // NEC protocol
    generate_logic_in_us(0, 9152);   // 9.152ms
    generate_logic_in_us(1, 4576);   // 4.576ms
    for (uint16_t i = 0; i < 32; i++) // 32 bits data
    {
        
        if (((data >> i) & 0x0001) == 0)
        {
            generate_logic_in_us(0, 572);
            generate_logic_in_us(1, 572);
//            DEBUG_INFO ("bit%d:0\r\n", i); // khong dc in log trong nay,tru luc debug tung bit 1
        }
        else
        {
            generate_logic_in_us(0, 572);
            generate_logic_in_us(1, 1716);
//            DEBUG_INFO ("bit%d:1\r\n", i);
        }
    }
    
    generate_logic_in_us(0, 572);
}

typedef enum 
{
    LOOCKING_FOR_FRAME,
    PREPRARE_START,
    HANDLING_DATA,
    CLOSE_FRAME
} data_state_t;

data_state_t data_state_now;


void decode_nec_protocol (uint16_t* data_rev)
{
    static bool start_frame = false;
    static uint16_t count_now = 0;
    static uint16_t first_pulse_data; 
//    uint16_t data = 0;
    uint32_t sample_time = 0; 
    
    if (!start_frame)
    {
        if (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10) == 0)
        {
            start_frame = true;
            data_state_now = LOOCKING_FOR_FRAME;
            count_now = 0;
        }
    }
    else
    {
        count_now++;
        switch (data_state_now)
        {
            case LOOCKING_FOR_FRAME:
                if (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10) == 1)
                {
                    sample_time = count_now / 7; // = 50
                    *data_rev = 0;
                    data_state_now = PREPRARE_START;
                }
            break;
            case PREPRARE_START:
                if (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10)== 0)
                {
                    while (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10) == 0);
                    first_pulse_data = count_now;
                    data_state_now = HANDLING_DATA;
                }
            break;
            case HANDLING_DATA:
                if (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10) == 0)
                {
                    if (count_now - first_pulse_data < (sample_time + 1))
                    {
                        *data_rev = *data_rev << 1;
                        //ghi bit 0
                    }
                    else if (count_now - first_pulse_data < (sample_time * 3 +1 ))
                    {
                        *data_rev = (*data_rev << 1) & 0x0001;
                        // ghi bit 1
                    }
                    else if (count_now - first_pulse_data > (sample_time * 5 +1))
                    {
                        data_state_now = CLOSE_FRAME;
                    }
                    while (LL_GPIO_IsInputPinSet(GPIOA, GPIO_PIN_10) == 0);
                    first_pulse_data = count_now;
                }
                
            break;
            case CLOSE_FRAME:
                // reset trang thai
                start_frame = false;
                if (data_rev != 0)
                {
                    DEBUG_WARN ("data_receive: %d\r\n", *data_rev);
                }
                
                // put out data
            break;
            default:
            break;
        }
    }
}

void UART_CharReception_Callback(void)
{
    //WE NEED POLL APP_CLI HERE
    uint8_t rev_char;
//    rev_char = LL_USART_ReceiveData8(USART2);
//    app_led_blink(0, 100, 2);
//    app_cli_poll(rev_char);
    lwrb_write (&lwrb_cli, &rev_char, 1);
}

