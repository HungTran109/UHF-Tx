#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#if 1

#include "app_debug.h"
#define DEBUG_RTT               1
#define PRINTF_OVER_RTT             DEBUG_RTT
#define PRINTF_OVER_UART            (!PRINTF_OVER_RTT)


#include "app_cli.h"
#include "app_shell.h"
#if PRINTF_OVER_RTT
#include "SEGGER_RTT.h"
#endif
#include "tim.h"
#include "app_uhf.h"
#if PRINTF_OVER_RTT
int rtt_custom_printf(const char *format, ...)
{
    int r;
    va_list ParamList;

    va_start(ParamList, format);
    r = SEGGER_RTT_vprintf(0, format, &ParamList);
    va_end(ParamList);
    
    return r;
}
#else
#define rtt_custom_printf           DEBUG_RAW
#endif

static shell_context_struct m_user_context;
static int32_t cli_reset_system(p_shell_context_t context, int32_t argc, char **argv);
static int32_t set_echo (p_shell_context_t context, int32_t argc, char **argv);
static int32_t set_freq (p_shell_context_t context, int32_t argc, char **argv);
#if 0
static int32_t cli_output_pwm_duty(p_shell_context_t context, int32_t argc, char **argv);
#endif /* DTG02V2 */
#if(TEST_PULSE)
extern uint32_t *get_pulse_counter(void);
#endif

static const shell_command_context_t cli_command_table[] = 
{
    {"reset",           "\treset: reset system\r\n",                            cli_reset_system,                           0},
    {"set_echo",        "\tset_echo: set echo level\r\n",                       set_echo,                                   2},
    {"set_freq",       "\tset_freq: set frequence level\r\n",                  set_freq,                                   1},
};

void app_cli_puts(uint8_t *buf, uint32_t len)
{
    SEGGER_RTT_Write(0, buf, len);
}

void app_cli_poll()
{
    app_shell_task();
}


extern uint8_t get_debug_rx_data(void);

void app_cli_gets(uint8_t *buf, uint32_t len)
{
#if PRINTF_OVER_RTT
    for (uint32_t i = 0; i < len; i++)
    {
        buf[i] = 0xFF;
    }
        
    if (!SEGGER_RTT_HASDATA(0))
    {
        return;
    }
    
    int read = SEGGER_RTT_Read(0, buf, len);
    if (read > 0 && read < len)
    {
        for (uint32_t i = read; i < len; i++)
        {
            buf[i] = 0xFF;
        }
    }
#endif
}


void app_cli_start()
{
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   app_cli_puts,
                   app_cli_gets,
                   rtt_custom_printf,
                   ">",
                   false);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task();
}

/* Reset System */
static int32_t cli_reset_system(p_shell_context_t context, int32_t argc, char **argv)
{
    DEBUG_INFO("System reset\r\n");
    NVIC_SystemReset();
    return 0;
}
char print_msg[64];

static int32_t set_echo (p_shell_context_t context, int32_t argc, char **argv)
{
//    uint8_t ratio, delay;
//    ratio = gsm_utilities_get_number_from_string(0, argv[1]);
//    delay = gsm_utilities_get_number_from_string(0, argv[2]);
//    set_echo_level (ratio, delay);
    return 0;
}

static int32_t set_freq (p_shell_context_t context, int32_t argc, char **argv)
{
    uint32_t freq = atoi(argv[1]);
    DEBUG_INFO("Set UHF Freq to %u\r\n", freq);
//    if (freq == 1)
//    {
//        app_led_blink(0, 100, 5);
//    }
//    else if (freq == 2)
//    {
//        app_led_blink(0, 100, 3);
//    }
    set_active_freq(freq);
    return 0;
}



#endif /* APP_CLI_ENABLE */
