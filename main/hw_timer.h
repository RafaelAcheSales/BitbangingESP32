#ifndef __HW_TIMER_H__
#define __HW_TIMER_H__

ICACHE_FLASH_ATTR
void hw_timer_init(void);
ICACHE_FLASH_ATTR
void hw_timer_set_func(void (*user_hw_timer_cb_set)(void));
ICACHE_FLASH_ATTR
void hw_timer_arm(uint32 val, bool req);
ICACHE_FLASH_ATTR
void hw_timer_disarm(void);

#endif  /* __HW_TIMER_H__ */
