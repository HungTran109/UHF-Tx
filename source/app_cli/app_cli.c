/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "app_spi_flash.h"
//#include "app_drv_spi.h"
//#include "utilities.h"
//#include "esp_log.h"
//#include "esp_system.h"
//#include "app_flash.h"
//#include "app_ota.h"
//#include "app_mqtt.h"
//#include "DataDefine.h"
//#include "app_aes.h"
//#include "base64.h"
//#include "app_audio.h"
#include "main.h"
#include "app_uhf.h"
#include "app_led.h"
//#include "app_debug.h"
//#include "diskio.h"

//static const char *TAG = "cli";
//extern app_flash_drv_t m_spi_flash;
//extern void send_test_cmd (void);
static app_cli_cb_t *m_cb;

static int32_t system_reset (p_shell_context_t context, int32_t argc, char **argv);
static int32_t set_echo (p_shell_context_t context, int32_t argc, char **argv);
//static int32_t get_build_timestamp (p_shell_context_t context, int32_t argc, char **argv);
static int32_t set_freq (p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] =
{
		 {"reset", "\treset: Reset stm32\r\n", system_reset, 0},
         {"set_echo", "\tset_echo: set echo level\r\n", set_echo, 2},
         {"set_freq", "\tset_freq: set frequence level\r\n", set_freq, 1}
//		 {"build", "\tbuild: Get firmware build timestamp\r\n", get_build_timestamp, 0},
};
static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start (app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   "",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}


static int32_t system_reset(p_shell_context_t context, int32_t argc, char **argv)
{
//	NVIC_SystemReset ();
	return 0;
}

uint32_t gsm_utilities_get_number_from_string(uint16_t begin_index, char *buffer)
{
    // assert(buffer);

    uint32_t value = 0;
    uint16_t tmp = begin_index;
    uint32_t len = strlen(buffer);
    while (buffer[tmp] && tmp < len)
    {
        if (buffer[tmp] >= '0' && buffer[tmp] <= '9')
        {
            value *= 10;
            value += buffer[tmp] - 48;
        }
        else
        {
            break;
        }
        tmp++;
    }

    return value;
}

char print_msg[64];

static int32_t set_echo (p_shell_context_t context, int32_t argc, char **argv)
{
    uint8_t ratio, delay;
    ratio = gsm_utilities_get_number_from_string(0, argv[1]);
    delay = gsm_utilities_get_number_from_string(0, argv[2]);
    set_echo_level (ratio, delay);
    return 0;
}

static int32_t set_freq (p_shell_context_t context, int32_t argc, char **argv)
{
    uint32_t freq = 0;
    freq = gsm_utilities_get_number_from_string(0, argv[1]);
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
//static int32_t get_build_timestamp (p_shell_context_t context, int32_t argc, char **argv)
//{
//	DEBUG_INFO("Build %s %s\r\n", __DATE__, __TIME__);
//	return 0;
//}
