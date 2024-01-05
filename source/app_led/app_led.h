/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>
#include <stdbool.h>

#define LED_APP_RGB 0
#define LED_NET_RGB 1
#define LED_APP_1_BLUE 2
#define LED_APP_2_GREEN 3
#define LED_APP_3_GREEN 4

#define LED_ON_FOREVER          UINT32_MAX
#define LED_BLINK_FOREVER	UINT32_MAX



#define RED 0
#define GREEN 1
#define BLUE 2

#define GRN GREEN
#define BLU BLUE

enum led_color {
	LED_COLOR_OFF, /* 000 */
	LED_COLOR_RED, /* 001 */
	LED_COLOR_GREEN, /* 010 */
	LED_COLOR_YELLOW, /* 011 */
	LED_COLOR_BLUE, /* 100 */
	LED_COLOR_MAGENTA, /* 101 */
	LED_COLOR_CYAN, /* 110 */
	LED_COLOR_WHITE, /* 111 */
	LED_COLOR_NUM,
};



/*user define LED name*/
#define LED_POWER 0	
#define LED_BLE	1
#define LED_ON LED_COLOR_WHITE

#define LED_BLINK true
#define LED_SOLID false

#define LED_UNIT_MAX 5


enum gpio_pull_cfg
{
    GPIO_PULL_NOPULL,
    GPIO_PULL_PULLUP,
    GPIO_PULL_PULLDOWN,
    GPIO_PULL_PULLUPDOWN,
    GPIO_PULL_FLOATTING
};


typedef uint8_t (*p_init_gpio_output)(enum gpio_pull_cfg pull_cfg, uint8_t led_pin);
typedef void (*p_func_set_gpio)(uint32_t port_num,uint32_t pin_num, uint8_t state);
typedef void (*p_func_reset_gpio)(void);
typedef void (*p_function_timer_expire_callback)(void *args);
typedef void(*p_function_led_done_callback)(void *args);
typedef struct app_led
{
    /* data */
    p_func_set_gpio set_gpio;
    p_func_reset_gpio reset_gpio;
    //p_function_timer_expire_callback timer_expire;
    
}gpio_callback_t;

/*This is for . file*/
enum led_type
{
    LED_MONOCHROME,
    LED_COLOR,
};
struct user_config
{
    bool blink;
    bool current_status;
    uint8_t blink_duration;
    uint8_t blink_current_tick;
    uint32_t blink_times;
    uint32_t on_times;
    enum led_color color;
    p_function_led_done_callback led_done;
};

struct led_unit_cfg
{
    uint16_t pin;
    uint16_t port;
    bool active_level;
    char *led_name;
    enum led_type unit_type;
    struct user_config user_cfg;
};

/**
 * @brief Set the state of a given LED unit to blink.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
 *			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 */
int app_led_blink(uint8_t led_unit, uint8_t on_off_duration, uint32_t blink_times);

/**
 * @brief Turn the given LED unit on.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
*			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 */
int app_led_on(uint8_t led_unit, uint32_t turn_on_time, ...);

/**
 * @brief Set the state of a given LED unit to off.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *		Using a LED unit assigned to another core will do nothing and return 0.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @return		0 on success
 */
int app_led_off(uint8_t led_unit);

/**
 * @brief Initialise the LED module
 *
 * @note This will parse the .dts files and configure all LEDs.
 *
 * @return	0 on success
 */
int app_led_init(gpio_callback_t p_callback, struct led_unit_cfg *p_cfg, uint32_t leds_number);
/*
 * @FunctionName  	: app_led_blink_handler 
 * @Return         	: None
 * @Parameters 		: None
 * @Author		    : TueTD
 * @Datecreated	    : 01/11/2022
 * @Description		: Led blink polling
 * @Notes		    : Call this in tick 1ms task 
 *******************************************************************************/
void app_led_blink_handler(void);


#endif /* _LED_H_ */
