/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "soc.h"

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;


/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;

#define US_TO_RTC_TIMER_TICKS(t)          \
    ((t) ?          \
     (((t) > 0x35A) ?            \
      (((t) >> 2) * ((APB_CLK_FREQ >> 4) / 250000) + ((t)&0x3) * ((APB_CLK_FREQ >> 4) / 1000000)) : \
      (((t) *(APB_CLK_FREQ>>4)) / 1000000)) :    \
         0)


typedef enum {          // timer provided mode
    DIVDED_BY_1   = 1,  // timer clock
    DIVDED_BY_16  = 16,  // divided by 16
    DIVDED_BY_256 = 256,  // divided by 256
} TIMER_PREDIVED_MODE;

typedef enum {          // timer interrupt mode
    TM_LEVEL_INT = 1,   // level interrupt
    TM_EDGE_INT  = 0,   // edge interrupt
} TIMER_INT_MODE;

static void (* user_hw_timer_cb)(void) = NULL;

bool frc1_auto_load = false;

static bool IRAM_ATTR hw_timer_isr_cb(void *arg)
{
    // BaseType_t high_task_awoken = pdFALSE;
    // example_timer_info_t *info = (example_timer_info_t *) args;

    // uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    // /* Prepare basic event data that will be then sent back to task */
    // example_timer_event_t evt = {
    //     .info.timer_group = info->timer_group,
    //     .info.timer_idx = info->timer_idx,
    //     .info.auto_reload = info->auto_reload,
    //     .info.alarm_interval = info->alarm_interval,
    //     .timer_counter_value = timer_counter_value
    // };

    // if (!info->auto_reload) {
    //     timer_counter_value += info->alarm_interval * TIMER_SCALE;
    //     timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    // }

    // /* Now just send the event data back to the main program task */
    // xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    // return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR

    if (user_hw_timer_cb != NULL) {
        (*(user_hw_timer_cb))();
    }
}

ICACHE_FLASH_ATTR
void hw_timer_disarm(void)
{
    timer_pause(TIMER_GROUP_0, TIMER_0);
}

ICACHE_FLASH_ATTR
void hw_timer_arm(uint32 val, bool auto_reload)
{

    timer_set_auto_reload(TIMER_GROUP_0, TIMER_0, auto_reload);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, US_TO_RTC_TIMER_TICKS(52));
    timer_start(TIMER_GROUP_0, TIMER_0);
    
}

ICACHE_FLASH_ATTR
void hw_timer_set_func(void (* user_hw_timer_cb_set)(void))
{
    user_hw_timer_cb = user_hw_timer_cb_set;
}

ICACHE_FLASH_ATTR
static void hw_timer_init()
{
    timer_config_t config = {
        .divider = DIVDED_BY_16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }; // default clock source is APB

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);


    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);
}
