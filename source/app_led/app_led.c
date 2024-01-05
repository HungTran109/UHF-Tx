#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_led.h"
#include "app_debug.h"
/*This is for header file*/
static uint8_t leds_num;
static bool initialized;
static struct led_unit_cfg led_units[LED_UNIT_MAX];

//static p_init_gpio_output m_gpio_output_init = NULL;
static p_func_set_gpio m_gpio_set_state = NULL;
//static p_function_timer_expire_callback m_timer_expire = NULL;
//static p_func_reset_gpio m_reset_gpio = NULL;

/**
 * @brief Configures fields for a monochrome LED
 */
static int config_led_monochrome(struct led_unit_cfg led_config, uint8_t led_unit)
{
	//led_units[led_unit].type.mono = &leds[led];
  led_units[led_unit].unit_type = LED_MONOCHROME;
  led_units[led_unit].user_cfg.blink_duration = 1;
  led_units[led_unit].user_cfg.blink_current_tick = 0;
  led_units[led_unit].user_cfg.blink_times = 0;
  led_units[led_unit].user_cfg.current_status = false;
  led_units[led_unit].led_name = led_config.led_name;
  led_units[led_unit].pin = led_config.pin;
  led_units[led_unit].port = led_config.port;
  led_units[led_unit].active_level = led_config.active_level;
  DEBUG_INFO("App led config: index:%d - name:%s - pin:%d - port:%d - active level: %s", led_unit
                                                                      , led_units[led_unit].led_name
                                                                      , led_units[led_unit].pin
                                                                      , led_units[led_unit].port 
                                                                      , (led_units[led_unit].active_level)? "High":"Low");
   return 0;
}

/**
 * @brief Internal handling to set the status of a led unit
 */
static int led_set_int(uint8_t led_unit, enum led_color color)
{
  int ret = 0;
  if(led_units[led_unit].unit_type == LED_MONOCHROME)
  {
    if(color)
    {
      m_gpio_set_state(led_units[led_unit].port, led_units[led_unit].pin, led_units[led_unit].active_level);
    } 
    else 
    {
      m_gpio_set_state(led_units[led_unit].port, led_units[led_unit].pin,led_units[led_unit].active_level ? 0 : 1);
    }
  }
  return ret;
}
/**
 * @brief Periodically invoked by the timer to blink LEDs.
 */
void app_led_blink_handler(void)
{
  int ret = 0;
  //static bool on_phase;
  for (uint8_t i = 0; i < leds_num; i++)
  {
    if(led_units[i].user_cfg.blink)
    {
      if(led_units[i].user_cfg.current_status == false)
      {
        if(led_units[i].user_cfg.blink_current_tick++ >= led_units[i].user_cfg.blink_duration)
        {
          if(led_units[i].user_cfg.blink_times)
          {
            if(led_units[i].user_cfg.blink_times != LED_BLINK_FOREVER)
            {
              led_units[i].user_cfg.blink_times--;
              //DEBUG_INFO("Blink times:%d", led_units[i].user_cfg.blink_times);
            }
            led_units[i].user_cfg.blink_current_tick = 0;
            led_units[i].user_cfg.current_status = true; 
            ret = led_set_int(i, LED_COLOR_RED);
            if(ret)
            {
                DEBUG_ERROR("Error went set device colour\r\n");
            }
          }
        }
      }
      else
      {
        if(led_units[i].user_cfg.blink_current_tick++ >= led_units[i].user_cfg.blink_duration)
        {
            led_units[i].user_cfg.blink_current_tick = 0;
            led_units[i].user_cfg.current_status = false; 
            ret = led_set_int(i, LED_COLOR_OFF);
            if(ret)
            {
                DEBUG_ERROR("Error went set device colour\r\n");
            }
        }
      }
    }
    else 
    {
      if(led_units[i].user_cfg.on_times == LED_ON_FOREVER)
      {
        m_gpio_set_state(led_units[i].port, led_units[i].pin, led_units[i].active_level);
      }
      else
      {
        if(led_units[i].user_cfg.on_times)
        {
          led_units[i].user_cfg.on_times--;
          m_gpio_set_state(led_units[i].port, led_units[i].pin, led_units[i].active_level);
        }
        else
        {
         m_gpio_set_state(led_units[i].port, led_units[i].pin, led_units[i].active_level ? 0 :1);
        }
      } 
    }
  }
}

static int led_set(uint8_t led_unit, enum led_color color, uint32_t on_times, uint32_t blink_times, bool blink)
{
  int ret;
  if (!initialized)
  {
    return -1;
  }
  ret = led_set_int(led_unit, color);
  if (ret) 
  {
    return ret;
  }
  led_units[led_unit].user_cfg.blink = blink;
  led_units[led_unit].user_cfg.color = color;
  if(blink)
  {
    led_units[led_unit].user_cfg.blink_times = blink_times;
    led_units[led_unit].user_cfg.blink_duration = on_times;
    led_units[led_unit].user_cfg.on_times = 0;
  }
  else
  {
    led_units[led_unit].user_cfg.on_times = on_times * 2;
    led_units[led_unit].user_cfg.blink_times = 0; 
  }
  return 0;
}

int app_led_on(uint8_t led_unit, uint32_t turn_on_time, ...)
{
  if (led_units[led_unit].unit_type == LED_MONOCHROME) {
    return led_set(led_unit, LED_ON, turn_on_time, 0,LED_SOLID);
  }
  else
  {
	  DEBUG_ERROR("Failed to set LED\r\n");
      return -1;
  }
}

int app_led_blink(uint8_t led_unit, uint8_t on_off_duration, uint32_t blink_times)
{
  if (led_units[led_unit].unit_type == LED_MONOCHROME) {
    return led_set(led_unit, LED_ON, on_off_duration, blink_times, LED_BLINK);
  }
  else
  {
		//LOG_ERR("Failed to set led blink\r\n");
      return -1;
  }
}

int app_led_off(uint8_t led_unit)
{
  return led_set(led_unit, LED_COLOR_OFF, 0, 0, LED_SOLID);
}

int app_led_init(gpio_callback_t p_callback, struct led_unit_cfg *p_cfg, uint32_t leds_number)
{
    //int ret = 0;
    if(leds_number > LED_UNIT_MAX)
    {
        // TODO: Add log function :D
        DEBUG_WARN("[app_led]: MEM ERROR");
        return -1;
    }
    if (initialized) {
        return -1;
    }
    if(p_callback.set_gpio)
    {
        m_gpio_set_state = p_callback.set_gpio;
    }
    for(uint8_t i = 0; i < leds_number; i++)
    {
        config_led_monochrome(p_cfg[i], i);
        app_led_off(i);
    }
    leds_num = leds_number;
    initialized = true;
    return 0;
}
int app_is_led_on(uint8_t led_unit)
{
  if(led_units[led_unit].user_cfg.blink_times || led_units[led_unit].user_cfg.on_times)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/*This is for . file*/



