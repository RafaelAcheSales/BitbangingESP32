#ifndef __TTY_H__
#define __TTY_H__

#define TTY_NUM_DEV     3

/* Baudrates */
#define BAUD_2400       0
#define BAUD_4800       1
#define BAUD_9600       2
#define BAUD_19200      3
#define BAUD_38400      4
#define BAUD_57600      5
#define BAUD_115200     6
#define BAUD_230400     7

/* Code schemes */
#define CS_8N1          0
#define CS_8E1          1
#define CS_8O1          2
#define CS_8N2          3

/* Parity */
#define PARITY_EVEN     0
#define PARITY_ODD      1
#define PARITY_NONE     2

typedef void (*tty_func_t)(int tty, const char *msg,
                           int len, void *user_data);

ICACHE_FLASH_ATTR int tty_init(void);
ICACHE_FLASH_ATTR void tty_release(void);
ICACHE_FLASH_ATTR int tty_open(int tty, tty_func_t func, void *user_data);
ICACHE_FLASH_ATTR int tty_close(int tty);
ICACHE_FLASH_ATTR int tty_write(int tty, const unsigned char *data, int len);
ICACHE_FLASH_ATTR int tty_tx_fifo_size(int tty);
ICACHE_FLASH_ATTR void tty_set_baudrate(int tty, int baudrate);
ICACHE_FLASH_ATTR void tty_set_parity(int tty, int mode);

#endif  /* __TTY_H__ */
